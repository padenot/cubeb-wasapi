#include <stdio.h>
#include <assert.h>
#include <Mmdeviceapi.h>
#include <Audioclient.h>
#include <stdint.h>
#include <math.h>
#include "cubeb.h"
#include "speex_resampler/speex_resampler.h"

/* Hundreads of nanoseconds per millisecond. */
#define HNS_PER_MS 10000

/* Microseconds per second. */
#define US_PER_S 1000000

#define SAFE_CLOSE_HANDLE(handle) \
  if (handle) { CloseHandle(handle); }

#define ARRAY_LENGTH(array_) \
  (sizeof(array_) / sizeof(array_[0]))

#define LOG(...) fprintf(stderr, __VA_ARGS__); fprintf(stderr, "\n");

template <typename T>
void SafeRelease(T ** ptr)
{
  if (*ptr) {
    (*ptr)->Release();
    *ptr = nullptr;
  }
}

struct cubeb
{
  IMMDevice* device;
};

typedef void (*refill_function)(cubeb_stream * stm, float * data, long frame_needed);

struct cubeb_stream
{
  IAudioClient * client;
  IAudioRenderClient * render_client;
  IAudioClock * audio_clock;
  HANDLE shutdown_event;
  HANDLE refill_event;
  HANDLE thread;
  /* Maximum number of frames we can be requested in a callback. */
  uint32_t buffer_frame_count;
  /* Mixer pameters. We need to convert the input 
   * stream to this samplerate/channel layout. */
  cubeb_stream_params mix_params;
  cubeb_stream_params stream_params;
  cubeb_state_callback state_callback;
  cubeb_data_callback data_callback;
  void * user_ptr;
  /* Resampler instance. If this is !NULL, resampling should happen. */
  SpeexResamplerState * resampler;
  float * resampling_src_buffer;
  refill_function refill_function;
  /* Leftover frames handling. */
  uint32_t leftover_frame_count;
  uint32_t leftover_frame_size;
  float * leftover_frames;
  uint8_t bytes_per_frame;
  bool draining;
  bool need_upmix;
};

/* in theory, windows in shared mode always support float32, so this is unused */
void convert_float32_to_integer16(float* in, long samples, short* out)
{
  for (int i = 0; i < samples; i++) {
    float v = in[i] * 32768.0f;
    out[i] = static_cast<short>(max(-32768.0f, min(32767.0f, v)));
  }
}

void convert_integer16_to_float32(short * in, long samples, float * out)
{
  for (int i = 0; i < samples; i++) {
    out[i] = in[i] / 32768.0f;
  }
}

/* |out| has to be twice as long as |in| */
template<typename T>
void mono_to_stereo(T * in, long insamples, T * out)
{
  int j = 0;
  for (int i = 0; i < insamples; i++, j+=2) {
    out[j] = out[j+1] = out[i];
  }
}

template<typename T>
void stereo_to_mono(T * in, long insamples, T * out)
{
  for (int i = 0, int j = 0; i < insamples; i+=2, j++) {
    out[j] = in[i] + in[i+1];
  }
}

static size_t
frame_to_bytes(cubeb_stream * stm, size_t frames)
{
  return stm->bytes_per_frame * frames;
}

void
refill_with_resampling(cubeb_stream * stm, float * data, long needed)
{
  if (stm->need_upmix) {
    needed /= 2;
  }
  /* Round up the input frame count, so in the worst case,
  * we have a leftover unresampled frames at the end, that we can use
  * during the next iteration.*/
  float rate = static_cast<float>(stm->stream_params.rate) / stm->mix_params.rate;
  long got;

  long before_resampling = static_cast<long>(ceilf(rate * needed) + 1);

  long frame_request = before_resampling - stm->leftover_frame_count;
  size_t leftover_bytes = frame_to_bytes(stm, stm->leftover_frame_count);

  memcpy(stm->resampling_src_buffer, stm->leftover_frames, leftover_bytes);
  uint8_t * buffer_start = reinterpret_cast<uint8_t*>(stm->resampling_src_buffer) + leftover_bytes;

  got = stm->data_callback(stm, stm->user_ptr, buffer_start, frame_request);

  if (got != frame_request) {
    LOG("draining.");
    stm->draining = true;
  }

  uint32_t in_frames_bck, out_frames_bck;
  uint32_t in_frames = in_frames_bck = before_resampling;
  uint32_t out_frames = out_frames_bck = needed;

  speex_resampler_process_interleaved_float(stm->resampler,
                                            stm->resampling_src_buffer,
                                            &in_frames, data, &out_frames);

  stm->leftover_frame_count = in_frames_bck - in_frames;
  size_t unresampled_bytes = frame_to_bytes(stm, stm->leftover_frame_count);

  uint8_t * leftover_frames_start = reinterpret_cast<uint8_t*>(stm->resampling_src_buffer);
  leftover_frames_start += frame_to_bytes(stm, in_frames);

  assert(stm->leftover_frame_count <= stm->leftover_frame_size);
  memcpy(stm->leftover_frames, leftover_frames_start, unresampled_bytes);

  /* if this is not true, there will be glitches */
  assert(out_frames == out_frames_bck);

  if (stm->need_upmix) {
    mono_to_stereo(stm->resampling_src_buffer, needed, data);
  }
}

void
refill(cubeb_stream * stm, float * data, long needed)
{
  if (stm->need_upmix) {
    needed /= 2;
  }

  long got = stm->data_callback(stm, stm->user_ptr, data, needed);
  
  if (got != needed) {
    LOG("draining.");
    stm->draining = true;
  }
  
  if (stm->need_upmix) {
    /* get a little buffer to upmix to. */
    /* XXX we should be able to use the resampler buffer here if the output rate 
    * is at least half the size of the input rate. */
    float* tmp = (float*)malloc(needed * 2 * sizeof(float));
    mono_to_stereo(data, needed, tmp);
    memcpy(data, tmp, needed * 2 * sizeof(float));
    free(tmp);
  }
}

static DWORD __stdcall
wasapi_stream_render_loop(LPVOID stream)
{
  cubeb_stream * stm = static_cast<cubeb_stream *>(stream);

  bool is_playing = true;
  HANDLE wait_array[2] = {stm->shutdown_event, stm->refill_event};
  HRESULT hr;

  /* XXX add mmcss support to increase thread priority. */

  hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);
  if (FAILED(hr)) {
    LOG("could not initialize COM in render thread: %x", hr);
    return hr;
  }

  while (is_playing) {
    DWORD waitResult = WaitForMultipleObjects(ARRAY_LENGTH(wait_array), wait_array, FALSE, INFINITE);

    switch (waitResult) {
    case WAIT_OBJECT_0: { /* shutdown */
      is_playing = false;
      continue;
    }
    case WAIT_OBJECT_0 + 1: { /* refill */
      BYTE* data;
      UINT32 padding;
      long available;

      hr = stm->client->GetCurrentPadding(&padding);
      if (stm->draining) {
        if(padding == 0) {
          stm->state_callback(stm, stm->user_ptr, CUBEB_STATE_DRAINED);
          is_playing = false;
        }
        continue;
      }
      if (SUCCEEDED(hr)) {
        available = stm->buffer_frame_count - padding;
        hr = stm->render_client->GetBuffer(available, &data);
        float* data_f = reinterpret_cast<float*>(data);
        if (SUCCEEDED(hr)) {

          (*stm->refill_function)(stm, data_f, available);

          hr = stm->render_client->ReleaseBuffer(available, 0);
          if (FAILED(hr)) {
            LOG("failed to release buffer.");
            is_playing = false;
          }
        } else {
          LOG("failed to get buffer.");
          is_playing = false;
        }
      } else {
        LOG("failed to get current padding.");
        is_playing = false;
      }
    }
    break;
    default:
      LOG("case %d not handled in render loop.", waitResult);
      abort();
    };
  }

  if (FAILED(hr)) {
    stm->state_callback(stm, stm->user_ptr, CUBEB_STATE_STOPPED);
  }
  CoUninitialize();
  return 0;
}

void wasapi_stream_destroy(cubeb_stream * stm);

int wasapi_init(cubeb ** context, char const * context_name)
{
  HRESULT hr;
  IMMDeviceEnumerator *pEnumerator = NULL;

  *context = (cubeb *)calloc(1, sizeof(cubeb));

  hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);

  if (FAILED(hr)) {
    LOG("Could not init COM.");
    return CUBEB_ERROR;
  }

  hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), NULL, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&pEnumerator));

  if (FAILED(hr)) {
    LOG("Could not get device enumerator.");
    return CUBEB_ERROR;
  }

  /* eMultimedia ? */
  hr = pEnumerator->GetDefaultAudioEndpoint(eRender, eMultimedia, &(*context)->device);

  if (FAILED(hr)) {
    LOG("Could not get default audio endpoint.");
    return CUBEB_ERROR;
  }

  (*context)->device->AddRef();

  return CUBEB_OK;
}

void wasapi_destroy(cubeb * context)
{
  context->device->Release();
}

char const* wasapi_get_backend_id(cubeb * context)
{
  return "wasapi";
}

int wasapi_stream_init(cubeb * context, cubeb_stream ** stream, char const * stream_name,
                       cubeb_stream_params stream_params, unsigned int latency,
                       cubeb_data_callback data_callback,
                       cubeb_state_callback state_callback,
                       void * user_ptr)
{
  HRESULT hr;
  WAVEFORMATEX* mix_format;

  assert(context && stream);

  /* 30ms in shared mode is the minimum we can get when using WASAPI */
  if (latency < 30) {
    LOG("Latency too low: got %u (30ms minimum)", latency);
    return CUBEB_ERROR;
  }

  *stream = (cubeb_stream *)calloc(1, sizeof(cubeb_stream));

  assert(*stream);

  (*stream)->data_callback = data_callback;
  (*stream)->state_callback = state_callback;
  (*stream)->user_ptr = user_ptr;
  (*stream)->stream_params = stream_params;
  (*stream)->draining = false;
  (*stream)->need_upmix = false;
  (*stream)->leftover_frames = NULL;

  (*stream)->shutdown_event = CreateEventEx(NULL, NULL, 0, EVENT_MODIFY_STATE | SYNCHRONIZE);

  if (!(*stream)->shutdown_event) {
    LOG("Can't create the shutdown event, error: %d.", GetLastError());
    wasapi_stream_destroy(*stream);
    return CUBEB_ERROR;
  }

  (*stream)->refill_event = CreateEventEx(NULL, NULL, 0, EVENT_MODIFY_STATE | SYNCHRONIZE);

  if (!(*stream)->shutdown_event) {
    LOG("Can't create the refill event, error: %d.", GetLastError());
    wasapi_stream_destroy(*stream);
    return CUBEB_ERROR;
  }

  /* we might want to handle stream switch events at some point. */

  /* Get a client. We will get all other interfaces we need from this pointer. */
  hr = context->device->Activate(__uuidof(IAudioClient), CLSCTX_INPROC_SERVER, NULL, (void **)&(*stream)->client);

  if (FAILED(hr)) {
    LOG("Could not activate the device to get an audio client.");
    wasapi_stream_destroy(*stream);
    return CUBEB_ERROR;
  }

  /* We have to dinstinguish between the format the mixer uses, 
  * and the format the stream we want to play uses. */
  (*stream)->client->GetMixFormat(&mix_format);
  (*stream)->bytes_per_frame = static_cast<uint8_t>(mix_format->nBlockAlign);
  if (mix_format->wFormatTag == WAVE_FORMAT_PCM ||
    mix_format->wFormatTag == WAVE_FORMAT_EXTENSIBLE &&
    reinterpret_cast<WAVEFORMATEXTENSIBLE *>(mix_format)->SubFormat == KSDATAFORMAT_SUBTYPE_PCM) {
      if (mix_format->wBitsPerSample == 16) {
        assert(0 && "we don't support that on desktop.");
        (*stream)->mix_params.format = CUBEB_SAMPLE_S16NE;
      } else {
        assert(0 && "integer, but not 16bits?");
      }
  } else if (mix_format->wFormatTag == WAVE_FORMAT_IEEE_FLOAT ||
    mix_format->wFormatTag == WAVE_FORMAT_EXTENSIBLE &&
    reinterpret_cast<WAVEFORMATEXTENSIBLE *>(mix_format)->SubFormat == KSDATAFORMAT_SUBTYPE_IEEE_FLOAT) {
      (*stream)->mix_params.format = CUBEB_SAMPLE_FLOAT32NE;
  } else {
    assert(0 && "sample format not supported.");
  }

  (*stream)->mix_params.rate = mix_format->nSamplesPerSec;
  (*stream)->mix_params.channels = mix_format->nChannels;

  float resampling_rate = static_cast<float>((*stream)->stream_params.rate) / (*stream)->mix_params.rate;

  /* If we are playing a mono stream, we need to upmix to stereo.
   * For now, we assume that the output support stereo sound.
   * The alternative would be sad */
  assert(mix_format->nChannels == 2);

  (*stream)->need_upmix = mix_format->nChannels != (*stream)->stream_params.channels;

  if (resampling_rate != 1.0) {
    /* If we are playing a mono stream, we only resample one channel,
     * and copy it over, so we are always resampling the number
     * of channels of the stream, not the number of channels that WASAPI wants. */
    (*stream)->resampler = speex_resampler_init((*stream)->stream_params.channels,
                                                (*stream)->stream_params.rate,
                                                (*stream)->mix_params.rate,
                                                SPEEX_RESAMPLER_QUALITY_DESKTOP,
                                                NULL);
    /* Get a little buffer so we can store the leftover frames,
     * i.e. the samples not consumed by the resampler that we will end up
     * using next time the render callback is called. */
    (*stream)->leftover_frame_size = static_cast<uint32_t>(ceilf(1 / resampling_rate * 2));
    (*stream)->leftover_frames = (float*)malloc(frame_to_bytes(*stream, (*stream)->leftover_frame_size));
    if (!(*stream)->resampler) {
      wasapi_stream_destroy(*stream);
      return CUBEB_ERROR;
    }

    (*stream)->refill_function = &refill_with_resampling;
  } else {
    (*stream)->refill_function = &refill;
  }

  LOG("Mix format: %d", (*stream)->mix_params.format);
  LOG("Mix samplerate: %d", (*stream)->mix_params.rate);
  LOG("Mix channels: %d", (*stream)->mix_params.channels);

  hr = (*stream)->client->Initialize(AUDCLNT_SHAREMODE_SHARED, 
                                     AUDCLNT_STREAMFLAGS_EVENTCALLBACK | AUDCLNT_STREAMFLAGS_NOPERSIST, 
                                     latency*HNS_PER_MS, 
                                     0, 
                                     mix_format, 
                                     NULL);

  if (FAILED(hr)) {
    LOG("Unable to initialize audio client: %x.", hr);
    wasapi_stream_destroy(*stream);
    return CUBEB_ERROR;
  }

  hr = (*stream)->client->GetBufferSize(&(*stream)->buffer_frame_count);

  if (FAILED(hr)) {
    LOG("Could not get the buffer size from the client.");
    wasapi_stream_destroy(*stream);
    return CUBEB_ERROR;
  }

  /* If we are going to resample, we will end up needing a buffer
   * to resample from, because speex's resampler does not do
   * in-place processing. Of course we need to take the resampling 
   * factor and the channel layout into account. */
  if ((*stream)->resampler) {
    size_t frames_needed = static_cast<size_t>(resampling_rate * (*stream)->buffer_frame_count);
    
    if ((*stream)->need_upmix) {
      frames_needed /= 2;
    }
    (*stream)->resampling_src_buffer = (float*)malloc(frame_to_bytes(*stream, frames_needed));
  }

  hr = (*stream)->client->SetEventHandle((*stream)->refill_event);

  if (FAILED(hr)) {
    LOG("Could set the event handle for the client.");
    wasapi_stream_destroy(*stream);
    return CUBEB_ERROR;
  }

  hr = (*stream)->client->GetService(__uuidof(IAudioRenderClient), (void**)&(*stream)->render_client);

  if (FAILED(hr)) {
    LOG("Could not get the render client.");
    free(*stream);
    return CUBEB_ERROR;
  }

  hr = (*stream)->client->GetService(__uuidof(IAudioClock), (void**)&(*stream)->audio_clock);

  CoTaskMemFree(mix_format);

  return CUBEB_OK;
}

void wasapi_stream_destroy(cubeb_stream * stm)
{
  assert(stm);

  if (stm->thread) {
    SetEvent(stm->shutdown_event);
    WaitForSingleObject(stm->thread, INFINITE);
    CloseHandle(stm->thread);
    stm->thread = nullptr;
  }

  SAFE_CLOSE_HANDLE(stm->shutdown_event);
  SAFE_CLOSE_HANDLE(stm->refill_event);

  SafeRelease(&stm->client);
  SafeRelease(&stm->render_client);
  SafeRelease(&stm->audio_clock);

  if (stm->leftover_frames) {
    free(stm->leftover_frames);
    stm->leftover_frames = 0;
  }

  if (stm->resampling_src_buffer) {
    free(stm->resampling_src_buffer);
  }

  free(stm);
}

int wasapi_stream_start(cubeb_stream * stm)
{
  HRESULT hr;
  BYTE* data;
 
  assert(stm);

  /* Prefill with silence so it does not glitch. */
  hr = stm->render_client->GetBuffer(stm->buffer_frame_count, &data);
  if (FAILED(hr)) {
    LOG("failed to get buffer");
    return CUBEB_ERROR;
  }
  
  hr = stm->render_client->ReleaseBuffer(stm->buffer_frame_count, AUDCLNT_BUFFERFLAGS_SILENT);

  if (FAILED(hr)) {
    LOG("failed to get buffer");
    return CUBEB_ERROR;
  }

  stm->thread = CreateThread(NULL, 0, wasapi_stream_render_loop, stm, 0, NULL);
  if (stm->thread == NULL) {
    LOG("could not create wasapi render thread.");
    return CUBEB_ERROR;
  }

  hr = stm->client->Start();
  if (FAILED(hr)) {
    LOG("could not start the stream.");
    return CUBEB_ERROR;
  }

  stm->state_callback(stm, stm->user_ptr, CUBEB_STATE_STARTED);

  return CUBEB_OK;
}

int wasapi_stream_stop(cubeb_stream * stm)
{
  assert(stm);

  HRESULT hr;

  if (stm->shutdown_event) {
    SetEvent(stm->shutdown_event);
  }

  hr = stm->client->Stop();

  if (FAILED(hr)) {
    LOG("could not stop AudioClient");
  }

  if (stm->thread) {
    WaitForSingleObject(stm->thread, INFINITE);
    CloseHandle(stm->thread);
    stm->thread = NULL;
  }
  
  stm->state_callback(stm, stm->user_ptr, CUBEB_STATE_STOPPED);

  return CUBEB_OK;
}

int wasapi_stream_get_position(cubeb_stream * stm, uint64_t * position)
{
  assert(stm && position);

  UINT64 freq, pos;
  HRESULT hr;

  hr = stm->audio_clock->GetFrequency(&freq);

  if (FAILED(hr)) {
    LOG("failed to get audio clock frequency.");
    return CUBEB_ERROR;
  }

  /* We can get higher precision clock by querying a
   * hardware counter using the second parameter, if needed. */
  hr = stm->audio_clock->GetPosition(&pos, NULL);

  *position = static_cast<uint64_t>(static_cast<double>(pos) / freq * US_PER_S);

  return CUBEB_OK;
}

int wasapi_stream_get_latency(cubeb_stream * stm, uint32_t * latency)
{
  assert(stm && latency);

  /* The GetStreamLatency method only works if the
   * AudioClient has been initialized. */
  if (!stm->client) {
    return CUBEB_ERROR;
  } else {
    REFERENCE_TIME latency_hns;
    stm->client->GetStreamLatency(&latency_hns);
    *latency = static_cast<uint32_t>(latency_hns / HNS_PER_MS / 1000. * stm->stream_params.rate);
  }

  return CUBEB_OK;
}

struct cubeb_ops {
  int (* init)(cubeb ** context, char const * context_name);
  char const * (* get_backend_id)(cubeb * context);
  void (* destroy)(cubeb * context);
  int (* stream_init)(cubeb * context, cubeb_stream ** stream, char const * stream_name,
                      cubeb_stream_params stream_params, unsigned int latency,
                      cubeb_data_callback data_callback,
                      cubeb_state_callback state_callback,
                      void * user_ptr);
  void (* stream_destroy)(cubeb_stream * stream);
  int (* stream_start)(cubeb_stream * stream);
  int (* stream_stop)(cubeb_stream * stream);
  int (* stream_get_position)(cubeb_stream * stream, uint64_t * position);
  int (* stream_get_latency)(cubeb_stream * stream, uint32_t * latency);
};

static struct cubeb_ops const wasapi_ops = {
  /*.init =*/ wasapi_init,
  /*.get_backend_id =*/ wasapi_get_backend_id,
  /*.destroy =*/ wasapi_destroy,
  /*.stream_init =*/ wasapi_stream_init,
  /*.stream_destroy =*/ wasapi_stream_destroy,
  /*.stream_start =*/ wasapi_stream_start,
  /*.stream_stop =*/ wasapi_stream_stop,
  /*.stream_get_position =*/ wasapi_stream_get_position,
  /*.stream_get_latency =*/ wasapi_stream_get_latency
};

void interleaved_stereo_sin(float freq, int channels, float* data, long frames, int samplerate)
{
  static int sin_step[32] = {{0}};
  float W = static_cast<float>(2 * 3.1416);
  // LOG("frames: %ld, channels: %d", frames, channels);
  for (int sample = 0; sample < frames * channels; sample+=channels) {
    for (int channel = 0; channel < channels; channel++) {
      data[sample + channel] = sinf(freq * W * sin_step[channel]++ / samplerate);
      // LOG("[%d]: %f", sample+channel, data[sample + channel]);
    }
  }
} 

static long data_callback(cubeb_stream* stream, void* user_ptr, void* data, long frames)
{
  float* a = reinterpret_cast<float*>(data);

  interleaved_stereo_sin(440, stream->stream_params.channels, a, frames, stream->stream_params.rate);

  return frames;
}

static void state_callback(cubeb_stream* stream, void* data, cubeb_state state)
{
  LOG("state: %d", state);
}

int main(int argc, char* argv[])
{
  cubeb* ctx;
  cubeb_stream* stream;
  wasapi_init(&ctx, "context_name");
  LOG("meh.");
  cubeb_stream_params params;
  params.channels = 2;
  params.format = CUBEB_SAMPLE_FLOAT32NE;
  params.rate = 8000;
  if (wasapi_stream_init(ctx, &stream, "meh", params, 30, data_callback, state_callback, NULL) != CUBEB_OK) {
    LOG("KO");
  } else {
    LOG("OK");
  }
  wasapi_stream_start(stream);
  int i = 5;
  while(i) {
    getchar();
    i--;
    uint64_t pos;
    uint32_t latency;
    wasapi_stream_get_position(stream, &pos);
    wasapi_stream_get_latency(stream, &latency);
    LOG("pos: %lld, latency: %u", pos, latency);
  }
  wasapi_stream_stop(stream);
  getchar();
  wasapi_destroy(ctx);
  return 0;
}

