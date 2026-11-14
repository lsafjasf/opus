/*
   Cross-frame consistency checks for the Opus encoder and decoder.

   The harness uses deterministic PCM and public API calls only.  It keeps
   exact-state checks (reset replay, force-channel state, sample counts and
   range-coder state) separate from lossy quality checks (waveform delta,
   frame energy and stereo phase).
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <math.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "opus.h"

#define CROSS_MAX_PACKET 8192
#define CROSS_MAX_PACKETS 4096
#define CROSS_PI 3.14159265358979323846
#define CROSS_RMS_FLOOR 48.0
#define CROSS_PHASE_FLOOR 256.0

/* Lossy thresholds.  Exact-state checks below do not use these values. */
#define CROSS_MAX_ENERGY_DELTA_DB 4.0
#define CROSS_MAX_JUMP_DB 24.0
#define CROSS_MIN_LR_PHASE 0.25
#define CROSS_MIN_SIGNAL_CORR 0.72
#define CROSS_MAX_SIGNAL_RMS_PCT 65.0

typedef struct {
   unsigned char data[CROSS_MAX_PACKET];
   opus_int32 len;
   opus_uint32 range;
} CrossPacket;

typedef struct {
   int bitrate;
   int complexity;
   int vbr;
} CrossConfig;

typedef struct {
   int start_ms;
   int bitrate;
   int complexity;
   int signal;
   int dtx;
   int fec;
   int loss;
   int vbr;
} CrossSegment;

typedef struct {
   int Fs;
   int channels;
   short *pcm;
   int pcm_frames;
   CrossPacket *packets;
   int packet_count;
   int total_bytes;
   short *decoded;
   int decoded_frames;
} CrossRun;

typedef struct {
   double max_energy_delta_db;
   double min_lr_phase;
   double min_signal_corr;
   double max_signal_rms_pct;
   double max_jump_db;
} CrossMetrics;

typedef struct {
   long checks;
   long failures;
   int verbose;
   int json;
   int fail_fast;
   unsigned seed;
} CrossContext;

static const int cross_short_rates[] = {12000, 24000, 64000};
static const int cross_full_rates[] = {
   8000, 12000, 16000, 24000, 32000, 48000, 64000, 96000, 128000
};

static double cross_clamp(double v, double lo, double hi)
{
   if (v < lo) return lo;
   if (v > hi) return hi;
   return v;
}

static double cross_smoothstep(double x)
{
   x = cross_clamp(x, 0.0, 1.0);
   return x*x*(3.0 - 2.0*x);
}

static void cross_fail(CrossContext *ctx, const char *scenario,
                       const char *what, const char *fmt, ...)
{
   va_list ap;
   ctx->failures++;
   if (ctx->json) {
      printf("{\"status\":\"fail\",\"scenario\":\"%s\",\"check\":\"%s\","
             "\"detail\":\"", scenario, what);
      va_start(ap, fmt);
      vprintf(fmt, ap);
      va_end(ap);
      printf("\"}\n");
   } else {
      fprintf(stderr, "FAIL [%s] %s: ", scenario, what);
      va_start(ap, fmt);
      vfprintf(stderr, fmt, ap);
      va_end(ap);
      fprintf(stderr, "\n");
   }
   if (ctx->fail_fast) exit(1);
}

static void cross_pass(CrossContext *ctx, const char *scenario,
                       const char *what, const char *detail)
{
   ctx->checks++;
   if (ctx->verbose) {
      if (ctx->json)
         printf("{\"status\":\"pass\",\"scenario\":\"%s\",\"check\":\"%s\","
                "\"detail\":\"%s\"}\n", scenario, what, detail);
      else
         printf("PASS [%s] %s: %s\n", scenario, what, detail);
   }
}

static void cross_assert(CrossContext *ctx, const char *scenario,
                         const char *what, int condition,
                         const char *fmt, ...)
{
   char detail[256];
   va_list ap;
   va_start(ap, fmt);
   vsnprintf(detail, sizeof(detail), fmt, ap);
   va_end(ap);
   if (condition)
      cross_pass(ctx, scenario, what, detail);
   else
      cross_fail(ctx, scenario, what, "%s", detail);
}

enum {
   CROSS_PCM_MIXED = 0,
   CROSS_PCM_DTX = 1,
   CROSS_PCM_SPEECH = 2,
   CROSS_PCM_STEADY = 3
};

static double cross_voice_wave(double t, unsigned seed)
{
   double f0;
   double phase;
   double vibrato;
   vibrato = sin(2*CROSS_PI*(0.61 + (seed%7)*0.013)*t);
   f0 = 92.0 + 24.0*vibrato + (seed%11);
   phase = 2*CROSS_PI*f0*t + 0.35*sin(2*CROSS_PI*3.1*t);
   return 0.46*sin(phase)
         +0.23*sin(2*phase + 0.13)
         +0.11*sin(3*phase + 0.37)
         +0.05*sin(5*phase + 0.71);
}

static double cross_music_wave(double t, unsigned seed)
{
   double chirp;
   chirp = 2*CROSS_PI*(0.35*t*t + (35 + (seed%5)*11)*t);
   return 0.28*sin(2*CROSS_PI*311.0*t + 0.2)
         +0.19*sin(2*CROSS_PI*733.0*t + 0.7)
         +0.12*sin(2*CROSS_PI*1499.0*t + 1.1)
         +0.08*sin(chirp)
         +0.025*sin(2*CROSS_PI*67.0*t);
}

static double cross_pcm_sample(int kind, double t, unsigned seed, int channel)
{
   double l;
   double r;
   double music_env;
   double voice_env;
   if (kind == CROSS_PCM_DTX) {
      if (t >= 0.50 && t < 1.50)
         return 0.0;
      l = cross_music_wave(t, seed);
      r = 0.70*l + 0.30*cross_music_wave(t + 0.0031, seed + 19);
      return channel ? r : l;
   }
   if (kind == CROSS_PCM_SPEECH) {
      double env = 0.55 + 0.45*sin(2*CROSS_PI*0.7*t);
      env = 0.25 + 0.75*cross_smoothstep(env);
      l = env*cross_voice_wave(t, seed);
      r = 0.68*l + 0.32*env*cross_voice_wave(t + 0.0027, seed + 23);
      return channel ? r : l;
   }
   if (kind == CROSS_PCM_STEADY) {
      double env = 0.88 + 0.12*sin(2*CROSS_PI*0.31*t);
      l = env*cross_music_wave(t, seed);
      r = 0.72*l + 0.28*env*cross_music_wave(t + 0.0029, seed + 17);
      return channel ? r : l;
   }
   /* Music -> speech -> music, with smooth rather than step envelopes. */
   music_env = 1.0;
   voice_env = 0.0;
   if (t > 0.62 && t < 0.78) {
      double x = (t - 0.62)/0.16;
      music_env = 1.0 - cross_smoothstep(x);
      voice_env = cross_smoothstep(x);
   } else if (t >= 0.78 && t < 1.22) {
      music_env = 0.0;
      voice_env = 1.0;
   } else if (t >= 1.22 && t < 1.38) {
      double x = (t - 1.22)/0.16;
      music_env = cross_smoothstep(x);
      voice_env = 1.0 - cross_smoothstep(x);
   }
   l = 0.72*music_env*cross_music_wave(t, seed)
       + 0.90*voice_env*cross_voice_wave(t, seed + 41);
   r = 0.62*l
       + 0.23*music_env*cross_music_wave(t + 0.0041, seed + 11)
       + 0.17*voice_env*cross_voice_wave(t + 0.0037, seed + 59);
   return channel ? r : l;
}

static short *cross_make_pcm(int frames, int channels, int kind, unsigned seed)
{
   short *pcm;
   int i;
   int c;
   pcm = (short *)calloc((size_t)frames*channels, sizeof(*pcm));
   if (pcm == NULL) {
      fprintf(stderr, "out of memory\n");
      exit(1);
   }
   for (i = 0; i < frames; ++i) {
      double t = (double)i/48000.0;
      for (c = 0; c < channels; ++c) {
         double v = 9000.0*cross_pcm_sample(kind, t, seed, c);
         if (v > 32767.0) v = 32767.0;
         if (v < -32768.0) v = -32768.0;
         pcm[(size_t)i*channels + c] = (short)floor(v + 0.5);
      }
   }
   return pcm;
}

static int cross_run_init(CrossRun *run, int Fs, int channels, int frames,
                          int kind, unsigned seed)
{
   int max_packets = frames/(Fs/400) + 16;
   memset(run, 0, sizeof(*run));
   run->Fs = Fs;
   run->channels = channels;
   run->pcm_frames = frames;
   run->pcm = cross_make_pcm(frames, channels, kind, seed);
   run->packets = (CrossPacket *)calloc((size_t)max_packets,
                                        sizeof(*run->packets));
   run->decoded = (short *)calloc((size_t)(frames + Fs/5)*channels,
                                  sizeof(*run->decoded));
   if (run->packets == NULL || run->decoded == NULL) {
      fprintf(stderr, "out of memory\n");
      exit(1);
   }
   return 0;
}

static void cross_run_clear(CrossRun *run)
{
   free(run->pcm);
   free(run->packets);
   free(run->decoded);
   memset(run, 0, sizeof(*run));
}

static void cross_apply_segment(OpusEncoder *enc, const CrossSegment *seg)
{
   opus_encoder_ctl(enc, OPUS_SET_BITRATE(seg->bitrate));
   opus_encoder_ctl(enc, OPUS_SET_COMPLEXITY(seg->complexity));
   opus_encoder_ctl(enc, OPUS_SET_SIGNAL(seg->signal));
   opus_encoder_ctl(enc, OPUS_SET_DTX(seg->dtx));
   opus_encoder_ctl(enc, OPUS_SET_INBAND_FEC(seg->fec));
   opus_encoder_ctl(enc, OPUS_SET_PACKET_LOSS_PERC(seg->loss));
   opus_encoder_ctl(enc, OPUS_SET_VBR(seg->vbr));
}

static int cross_segment_at(const CrossSegment *segments, int count,
                            int time_ms)
{
   int i;
   int selected = 0;
   for (i = 0; i < count; ++i) {
      if (segments[i].start_ms <= time_ms)
         selected = i;
   }
   return selected;
}

static int cross_encode_with_encoder(CrossRun *run, OpusEncoder *enc,
                                     int chunk_frames,
                                     const CrossSegment *segments,
                                     int segment_count)
{
   int pos;
   int last_segment = -1;
   run->packet_count = 0;
   run->total_bytes = 0;
   for (pos = 0; pos < run->pcm_frames; ) {
      CrossPacket *packet;
      int time_ms;
      int segment;
      int frame = chunk_frames;
      int len;
      if (frame > run->pcm_frames - pos)
         frame = run->pcm_frames - pos;
      time_ms = pos*1000/run->Fs;
      segment = cross_segment_at(segments, segment_count, time_ms);
      if (segment != last_segment) {
         cross_apply_segment(enc, &segments[segment]);
         last_segment = segment;
      }
      if (run->packet_count >= CROSS_MAX_PACKETS) {
         return OPUS_BUFFER_TOO_SMALL;
      }
      packet = &run->packets[run->packet_count];
      len = opus_encode(enc, &run->pcm[(size_t)pos*run->channels],
                        frame, packet->data, CROSS_MAX_PACKET);
      if (len <= 0 || len > CROSS_MAX_PACKET) {
         return len < 0 ? len : OPUS_INTERNAL_ERROR;
      }
      opus_encoder_ctl(enc, OPUS_GET_FINAL_RANGE(&packet->range));
      packet->len = len;
      run->packet_count++;
      run->total_bytes += len;
      pos += frame;
   }
   return OPUS_OK;
}

static int cross_encode_run(CrossRun *run, int application,
                            int chunk_frames, const CrossSegment *segments,
                            int segment_count)
{
   OpusEncoder *enc;
   int err;
   int ret;
   enc = opus_encoder_create(run->Fs, run->channels, application, &err);
   if (enc == NULL || err != OPUS_OK) {
      if (enc != NULL) opus_encoder_destroy(enc);
      return OPUS_INTERNAL_ERROR;
   }
   ret = cross_encode_with_encoder(run, enc, chunk_frames,
                                   segments, segment_count);
   opus_encoder_destroy(enc);
   return ret;
}

static int cross_decode_run(CrossRun *run)
{
   OpusDecoder *dec;
   int err;
   int i;
   int max_frame = run->Fs/5;
   int total = 0;
   dec = opus_decoder_create(run->Fs, run->channels, &err);
   if (dec == NULL || err != OPUS_OK) {
      if (dec != NULL) opus_decoder_destroy(dec);
      return OPUS_INTERNAL_ERROR;
   }
   for (i = 0; i < run->packet_count; ++i) {
      int got;
      CrossPacket *packet = &run->packets[i];
      got = opus_decode(dec, packet->data, packet->len,
                        &run->decoded[(size_t)total*run->channels],
                        max_frame, 0);
      if (got <= 0) {
         opus_decoder_destroy(dec);
         return got < 0 ? got : OPUS_INTERNAL_ERROR;
      }
      total += got;
   }
   opus_decoder_destroy(dec);
   run->decoded_frames = total;
   return OPUS_OK;
}

static double cross_rms(const short *pcm, int frames, int channels,
                        int channel)
{
   double sum = 0.0;
   int i;
   if (frames <= 0) return 0.0;
   for (i = 0; i < frames; ++i) {
      double v = pcm[(size_t)i*channels + channel];
      sum += v*v;
   }
   return sqrt(sum/frames);
}

static double cross_corr(const short *a, const short *b, int frames,
                         int channels, int channel)
{
   double ab = 0.0;
   double aa = 0.0;
   double bb = 0.0;
   int i;
   for (i = 0; i < frames; ++i) {
      double av = a[(size_t)i*channels + channel];
      double bv = b[(size_t)i*channels + channel];
      ab += av*bv;
      aa += av*av;
      bb += bv*bv;
   }
   if (aa < 1e-9 || bb < 1e-9)
      return 1.0;
   return ab/sqrt(aa*bb);
}

static double cross_db_ratio(double a, double b)
{
   if (a < CROSS_RMS_FLOOR) a = CROSS_RMS_FLOOR;
   if (b < CROSS_RMS_FLOOR) b = CROSS_RMS_FLOOR;
   return 20.0*log10(a/b);
}

static void cross_measure_run_ex(const CrossRun *run, int window_frames,
                                 const int *skip_ms, int skip_count,
                                 int guard_ms, CrossMetrics *m)
{
   int pos;
   double previous = -1.0;
   memset(m, 0, sizeof(*m));
   m->min_lr_phase = 1.0;
   m->min_signal_corr = 1.0;
   for (pos = 0; pos + window_frames <= run->decoded_frames;
        pos += window_frames) {
      const short *frame = &run->decoded[(size_t)pos*run->channels];
      int time_ms = pos*1000/run->Fs;
      int s;
      int skip = 0;
      double left = cross_rms(frame, window_frames, run->channels, 0);
      double right = run->channels > 1
                   ? cross_rms(frame, window_frames, run->channels, 1)
                   : left;
      for (s = 0; s < skip_count; ++s) {
         if (time_ms >= skip_ms[s] && time_ms < skip_ms[s] + guard_ms) {
            skip = 1;
            break;
         }
      }
      if (skip) {
         previous = -1.0;
         continue;
      }
      if (previous >= 0.0) {
         double jump = fabs(cross_db_ratio(left, previous));
         if (jump > m->max_jump_db)
            m->max_jump_db = jump;
      }
      previous = left;
      if (left >= CROSS_PHASE_FLOOR && right >= CROSS_PHASE_FLOOR) {
         double phase;
         double ab = 0.0;
         double aa = 0.0;
         double bb = 0.0;
         int i;
         for (i = 0; i < window_frames; ++i) {
            double l = frame[(size_t)i*run->channels];
            double r = frame[(size_t)i*run->channels + 1];
            ab += l*r;
            aa += l*l;
            bb += r*r;
         }
         phase = ab/sqrt(aa*bb + 1e-9);
         if (phase < m->min_lr_phase)
            m->min_lr_phase = phase;
      }
   }
}

static void cross_measure_run(const CrossRun *run, int window_frames,
                              CrossMetrics *m)
{
   cross_measure_run_ex(run, window_frames, NULL, 0, 0, m);
}

static void cross_compare_runs(const CrossRun *a, const CrossRun *b,
                               int window_frames,
                               const int *transition_ms, int transition_count,
                               int guard_ms, CrossMetrics *m)
{
   int pos;
   int channel;
   memset(m, 0, sizeof(*m));
   m->min_lr_phase = 1.0;
   m->min_signal_corr = 1.0;
   for (pos = 0; pos + window_frames <= a->decoded_frames
                 && pos + window_frames <= b->decoded_frames;
        pos += window_frames) {
      const short *fa = &a->decoded[(size_t)pos*a->channels];
      const short *fb = &b->decoded[(size_t)pos*b->channels];
      double ea[2];
      double eb[2];
      int time_ms = pos*1000/a->Fs;
      int t;
      int skip = 0;
      for (t = 0; t < transition_count; ++t) {
         if (time_ms >= transition_ms[t]
             && time_ms < transition_ms[t] + guard_ms) {
            skip = 1;
            break;
         }
      }
      if (skip)
         continue;
      for (channel = 0; channel < a->channels; ++channel) {
         double corr;
         double delta_db;
         ea[channel] = cross_rms(fa, window_frames, a->channels, channel);
         eb[channel] = cross_rms(fb, window_frames, b->channels, channel);
         delta_db = fabs(cross_db_ratio(ea[channel], eb[channel]));
         if (delta_db > m->max_energy_delta_db)
            m->max_energy_delta_db = delta_db;
         if (ea[channel] >= CROSS_RMS_FLOOR ||
             eb[channel] >= CROSS_RMS_FLOOR) {
            corr = cross_corr(fa, fb, window_frames, a->channels, channel);
            if (corr < m->min_signal_corr)
               m->min_signal_corr = corr;
         }
      }
      {
         double diff = 0.0;
         double ref = 0.0;
         int i;
         for (i = 0; i < window_frames*a->channels; ++i) {
            double d = (double)fa[i] - fb[i];
            diff += d*d;
            ref += (double)fa[i]*fa[i];
         }
         if (ref > 1e-9) {
            double pct = 100.0*sqrt(diff/ref);
            if (pct > m->max_signal_rms_pct)
               m->max_signal_rms_pct = pct;
         }
      }
   }
}

static void cross_packet_stats(const CrossRun *run, int *min_len,
                               int *max_len)
{
   int i;
   *min_len = CROSS_MAX_PACKET;
   *max_len = 0;
   for (i = 0; i < run->packet_count; ++i) {
      if (run->packets[i].len < *min_len)
         *min_len = run->packets[i].len;
      if (run->packets[i].len > *max_len)
         *max_len = run->packets[i].len;
   }
}

static void cross_report_run(CrossContext *ctx, const char *scenario,
                             const CrossRun *run, int expected_frames)
{
   int min_len;
   int max_len;
   cross_packet_stats(run, &min_len, &max_len);
   cross_assert(ctx, scenario, "encoded-bytes",
                min_len >= 1 && max_len <= CROSS_MAX_PACKET,
                "packets=%d bytes=%d range=%d..%d",
                run->packet_count, run->total_bytes, min_len, max_len);
   cross_assert(ctx, scenario, "sample-count",
                run->decoded_frames == expected_frames,
                "decoded=%d expected=%d", run->decoded_frames,
                expected_frames);
}

static void cross_check_lossy_custom(CrossContext *ctx, const char *scenario,
                                     const CrossMetrics *m,
                                     double max_energy_db,
                                     double min_corr,
                                     double max_rms_pct);

static void cross_check_lossy(CrossContext *ctx, const char *scenario,
                              const CrossMetrics *m)
{
   cross_check_lossy_custom(ctx, scenario, m,
                            CROSS_MAX_ENERGY_DELTA_DB,
                            CROSS_MIN_SIGNAL_CORR,
                            CROSS_MAX_SIGNAL_RMS_PCT);
}

static void cross_check_lossy_custom(CrossContext *ctx, const char *scenario,
                                     const CrossMetrics *m,
                                     double max_energy_db,
                                     double min_corr,
                                     double max_rms_pct)
{
   cross_assert(ctx, scenario, "energy-match",
                m->max_energy_delta_db <= max_energy_db,
                "max=%.3f dB limit=%.3f dB",
                m->max_energy_delta_db, max_energy_db);
   cross_assert(ctx, scenario, "waveform-correlation",
                m->min_signal_corr >= min_corr,
                "min=%.4f limit=%.4f", m->min_signal_corr, min_corr);
   cross_assert(ctx, scenario, "waveform-rms-delta",
                m->max_signal_rms_pct <= max_rms_pct,
                "max=%.3f%% limit=%.3f%%",
                m->max_signal_rms_pct, max_rms_pct);
}

static void cross_check_phase(CrossContext *ctx, const char *scenario,
                              const CrossMetrics *m)
{
   cross_assert(ctx, scenario, "stereo-phase",
                m->min_lr_phase >= CROSS_MIN_LR_PHASE,
                "min=%.4f limit=%.4f",
                m->min_lr_phase, CROSS_MIN_LR_PHASE);
}

static void cross_test_steady_cbr(CrossContext *ctx, const CrossConfig *cfg)
{
   CrossRun run20;
   CrossRun run100;
   CrossMetrics metrics;
   CrossMetrics phase;
   CrossSegment segment;
   char scenario[96];
   int frames = 2*48000;
   int rc20;
   int rc100;
   int min20;
   int max20;
   int min100;
   int max100;
   snprintf(scenario, sizeof(scenario), "steady-cbr br=%d cx=%d",
            cfg->bitrate, cfg->complexity);
   memset(&segment, 0, sizeof(segment));
   segment.bitrate = cfg->bitrate;
   segment.complexity = cfg->complexity;
   segment.signal = OPUS_SIGNAL_MUSIC;
   segment.vbr = 0;
   cross_run_init(&run20, 48000, 2, frames, CROSS_PCM_STEADY, ctx->seed+1);
   cross_run_init(&run100, 48000, 2, frames, CROSS_PCM_STEADY, ctx->seed+1);
   rc20 = cross_encode_run(&run20, OPUS_APPLICATION_AUDIO,
                           48000/50, &segment, 1);
   rc100 = cross_encode_run(&run100, OPUS_APPLICATION_AUDIO,
                            48000/10, &segment, 1);
   cross_assert(ctx, scenario, "encode",
                rc20 == OPUS_OK && rc100 == OPUS_OK,
                "20ms=%s 100ms=%s", opus_strerror(rc20),
                opus_strerror(rc100));
   if (rc20 == OPUS_OK) rc20 = cross_decode_run(&run20);
   if (rc100 == OPUS_OK) rc100 = cross_decode_run(&run100);
   cross_report_run(ctx, scenario, &run20, frames);
   cross_report_run(ctx, scenario, &run100, frames);
   cross_packet_stats(&run20, &min20, &max20);
   cross_packet_stats(&run100, &min100, &max100);
   cross_assert(ctx, scenario, "cbr-packet-stability",
                min20 == max20 && min100 == max100,
                "20ms=%d..%d 100ms=%d..%d",
                min20, max20, min100, max100);
   cross_assert(ctx, scenario, "cbr-byte-total",
                abs(run20.total_bytes-run100.total_bytes) <= 64,
                "20ms=%d 100ms=%d tolerance=64 bytes",
                run20.total_bytes, run100.total_bytes);
   cross_compare_runs(&run20, &run100, 48000/50, NULL, 0, 0, &metrics);
   cross_measure_run(&run20, 48000/50, &phase);
   if (cfg->bitrate >= 16000)
      cross_check_lossy_custom(ctx, scenario, &metrics, 3.0, 0.72, 45.0);
   cross_check_phase(ctx, scenario, &phase);
   {
      double jump_limit = cfg->bitrate < 16000 ? 30.0 : CROSS_MAX_JUMP_DB;
      cross_assert(ctx, scenario, "energy-jump",
                   phase.max_jump_db <= jump_limit,
                   "max=%.3f dB limit=%.3f dB",
                   phase.max_jump_db, jump_limit);
   }
   cross_run_clear(&run20);
   cross_run_clear(&run100);
}

static void cross_test_chunking_switch(CrossContext *ctx,
                                       const CrossConfig *cfg)
{
   CrossRun run20;
   CrossRun run100;
   CrossMetrics compare;
   CrossMetrics phase;
   CrossSegment segments[3];
   char scenario[96];
   int frames = 2*48000;
   int i;
   int transitions[2] = {600, 1200};
   int saw_silk = 0;
   int saw_celt = 0;
   int rc20;
   int rc100;
   int high = cfg->bitrate > 64000 ? cfg->bitrate : 64000;
   snprintf(scenario, sizeof(scenario), "chunk-20-vs-100 br=%d cx=%d",
            cfg->bitrate, cfg->complexity);
   memset(segments, 0, sizeof(segments));
   segments[0].start_ms = 0;
   segments[0].bitrate = high;
   segments[0].complexity = cfg->complexity;
   segments[0].signal = OPUS_SIGNAL_MUSIC;
   segments[0].vbr = 1;
   segments[1].start_ms = 600;
   segments[1].bitrate = 12000;
   segments[1].complexity = cfg->complexity;
   segments[1].signal = OPUS_SIGNAL_VOICE;
   segments[1].vbr = 1;
   segments[2].start_ms = 1200;
   segments[2].bitrate = high;
   segments[2].complexity = cfg->complexity;
   segments[2].signal = OPUS_SIGNAL_MUSIC;
   segments[2].vbr = 1;
   cross_run_init(&run20, 48000, 2, frames, CROSS_PCM_MIXED, ctx->seed+2);
   cross_run_init(&run100, 48000, 2, frames, CROSS_PCM_MIXED, ctx->seed+2);
   rc20 = cross_encode_run(&run20, OPUS_APPLICATION_VOIP,
                           48000/50, segments, 3);
   rc100 = cross_encode_run(&run100, OPUS_APPLICATION_VOIP,
                            48000/10, segments, 3);
   cross_assert(ctx, scenario, "encode",
                rc20 == OPUS_OK && rc100 == OPUS_OK,
                "20ms=%s 100ms=%s", opus_strerror(rc20),
                opus_strerror(rc100));
   if (rc20 == OPUS_OK) rc20 = cross_decode_run(&run20);
   if (rc100 == OPUS_OK) rc100 = cross_decode_run(&run100);
   cross_report_run(ctx, scenario, &run20, frames);
   cross_report_run(ctx, scenario, &run100, frames);
   for (i = 0; i < run20.packet_count; ++i) {
      int config = run20.packets[i].data[0] >> 3;
      if (config < 16) saw_silk = 1;
      if (config >= 16) saw_celt = 1;
   }
   cross_assert(ctx, scenario, "silk-celt-coverage",
                saw_silk && saw_celt,
                "silk=%d celt=%d", saw_silk, saw_celt);
   cross_assert(ctx, scenario, "byte-total",
                abs(run20.total_bytes-run100.total_bytes)
                   <= run100.total_bytes/10 + 64,
                "20ms=%d 100ms=%d", run20.total_bytes, run100.total_bytes);
   cross_compare_runs(&run20, &run100, 48000/50, transitions, 2, 160,
                      &compare);
   cross_measure_run(&run20, 48000/50, &phase);
   cross_check_lossy(ctx, scenario, &compare);
   cross_check_phase(ctx, scenario, &phase);
   cross_assert(ctx, scenario, "energy-jump",
                phase.max_jump_db <= CROSS_MAX_JUMP_DB,
                "max=%.3f dB limit=%.3f dB",
                phase.max_jump_db, CROSS_MAX_JUMP_DB);
   cross_run_clear(&run20);
   cross_run_clear(&run100);
}

static void cross_test_short_celt(CrossContext *ctx, const CrossConfig *cfg)
{
   CrossRun run25;
   CrossRun run50;
   CrossMetrics compare;
   CrossMetrics phase;
   CrossSegment segment;
   char scenario[96];
   int frames = 48000*2/5;
   int i;
   int celt_packets = 0;
   int rc25;
   int rc50;
   snprintf(scenario, sizeof(scenario), "celt-2.5-vs-5ms br=%d cx=%d",
            cfg->bitrate, cfg->complexity);
   memset(&segment, 0, sizeof(segment));
   segment.bitrate = cfg->bitrate < 48000 ? 48000 : cfg->bitrate;
   segment.complexity = cfg->complexity;
   segment.signal = OPUS_SIGNAL_MUSIC;
   segment.vbr = 1;
   cross_run_init(&run25, 48000, 2, frames, CROSS_PCM_MIXED, ctx->seed+3);
   cross_run_init(&run50, 48000, 2, frames, CROSS_PCM_MIXED, ctx->seed+3);
   rc25 = cross_encode_run(&run25, OPUS_APPLICATION_RESTRICTED_LOWDELAY,
                           48000/400, &segment, 1);
   rc50 = cross_encode_run(&run50, OPUS_APPLICATION_RESTRICTED_LOWDELAY,
                           48000/200, &segment, 1);
   cross_assert(ctx, scenario, "encode",
                rc25 == OPUS_OK && rc50 == OPUS_OK,
                "2.5ms=%s 5ms=%s", opus_strerror(rc25),
                opus_strerror(rc50));
   if (rc25 == OPUS_OK) rc25 = cross_decode_run(&run25);
   if (rc50 == OPUS_OK) rc50 = cross_decode_run(&run50);
   cross_report_run(ctx, scenario, &run25, frames);
   cross_report_run(ctx, scenario, &run50, frames);
   for (i = 0; i < run25.packet_count; ++i) {
      int config = run25.packets[i].data[0] >> 3;
      if (config >= 16) celt_packets++;
   }
   cross_assert(ctx, scenario, "all-celt",
                celt_packets == run25.packet_count,
                "celt=%d packets=%d", celt_packets, run25.packet_count);
   cross_compare_runs(&run25, &run50, 48000/100, NULL, 0, 0, &compare);
   cross_measure_run(&run25, 48000/400, &phase);
   cross_check_lossy_custom(ctx, scenario, &compare, 3.0, 0.70, 80.0);
   cross_check_phase(ctx, scenario, &phase);
   cross_run_clear(&run25);
   cross_run_clear(&run50);
}

static void cross_test_dtx(CrossContext *ctx, const CrossConfig *cfg)
{
   CrossRun run20;
   CrossRun run100;
   CrossMetrics phase;
   CrossSegment segment;
   char scenario[96];
   int frames = 2*48000;
   int dtx20 = 0;
   int dtx100 = 0;
   int i;
   int rc20;
   int rc100;
   double active;
   double quiet;
   snprintf(scenario, sizeof(scenario), "dtx br=%d cx=%d",
            cfg->bitrate, cfg->complexity);
   memset(&segment, 0, sizeof(segment));
   segment.bitrate = cfg->bitrate;
   segment.complexity = cfg->complexity;
   segment.signal = OPUS_SIGNAL_VOICE;
   segment.dtx = 1;
   segment.vbr = 1;
   cross_run_init(&run20, 48000, 2, frames, CROSS_PCM_DTX, ctx->seed+4);
   cross_run_init(&run100, 48000, 2, frames, CROSS_PCM_DTX, ctx->seed+4);
   rc20 = cross_encode_run(&run20, OPUS_APPLICATION_VOIP,
                           48000/50, &segment, 1);
   rc100 = cross_encode_run(&run100, OPUS_APPLICATION_VOIP,
                            48000/10, &segment, 1);
   cross_assert(ctx, scenario, "encode",
                rc20 == OPUS_OK && rc100 == OPUS_OK,
                "20ms=%s 100ms=%s", opus_strerror(rc20),
                opus_strerror(rc100));
   if (rc20 == OPUS_OK) rc20 = cross_decode_run(&run20);
   if (rc100 == OPUS_OK) rc100 = cross_decode_run(&run100);
   cross_report_run(ctx, scenario, &run20, frames);
   cross_report_run(ctx, scenario, &run100, frames);
   for (i = 0; i < run20.packet_count; ++i)
      if (run20.packets[i].len == 1) dtx20++;
   for (i = 0; i < run100.packet_count; ++i)
      if (run100.packets[i].len <= 8) dtx100++;
   cross_assert(ctx, scenario, "dtx-packets",
                dtx20 > 0 && dtx100 > 0,
                "20ms=%d 100ms=%d", dtx20, dtx100);
   active = cross_rms(&run20.decoded[(size_t)(48000/5)*2],
                      48000/5, 2, 0);
   quiet = cross_rms(&run20.decoded[(size_t)(48000*7/10)*2],
                     48000*3/5, 2, 0);
   cross_assert(ctx, scenario, "dtx-comfort-noise",
                quiet <= active*0.25,
                "active=%.1f quiet=%.1f limit=25%%", active, quiet);
   cross_measure_run(&run20, 48000/50, &phase);
   cross_assert(ctx, scenario, "energy-jump",
                phase.max_jump_db <= 36.0,
                "max=%.3f dB limit=36.000 dB", phase.max_jump_db);
   cross_run_clear(&run20);
   cross_run_clear(&run100);
}

static int cross_decode_with_loss(CrossRun *run, int use_fec,
                                  int *fec_recovered, int *plc_frames,
                                  int *range_errors)
{
   OpusDecoder *dec;
   int err;
   int i;
   int total = 0;
   int pending = 0;
   int max_frame = run->Fs/5;
   int frame_samples = run->Fs/50;
   dec = opus_decoder_create(run->Fs, run->channels, &err);
   if (dec == NULL || err != OPUS_OK) {
      if (dec != NULL) opus_decoder_destroy(dec);
      return OPUS_INTERNAL_ERROR;
   }
   *fec_recovered = 0;
   *plc_frames = 0;
   *range_errors = 0;
   for (i = 0; i < run->packet_count; ++i) {
      CrossPacket *packet = &run->packets[i];
      int lost = (i % 13 == 5);
      if (lost) {
         pending++;
         continue;
      }
      if (pending) {
         if (use_fec && pending == 1
             && opus_packet_has_lbrr(packet->data, packet->len)) {
            int got = opus_decode(dec, packet->data, packet->len,
                                  &run->decoded[(size_t)total*run->channels],
                                  frame_samples, 1);
            if (got > 0) {
               total += got;
               (*fec_recovered)++;
            } else {
               got = opus_decode(dec, NULL, 0,
                                 &run->decoded[(size_t)total*run->channels],
                                 frame_samples, 0);
               if (got <= 0) {
                  opus_decoder_destroy(dec);
                  return got < 0 ? got : OPUS_INTERNAL_ERROR;
               }
               total += got;
               (*plc_frames)++;
            }
         } else {
            while (pending > 0) {
               int got = opus_decode(dec, NULL, 0,
                                     &run->decoded[(size_t)total*run->channels],
                                     frame_samples, 0);
               if (got <= 0) {
                  opus_decoder_destroy(dec);
                  return got < 0 ? got : OPUS_INTERNAL_ERROR;
               }
               total += got;
               (*plc_frames)++;
               pending--;
            }
         }
         pending = 0;
      }
      {
         int got = opus_decode(dec, packet->data, packet->len,
                               &run->decoded[(size_t)total*run->channels],
                               max_frame, 0);
         opus_uint32 decoded_range = 0;
         if (got <= 0) {
            opus_decoder_destroy(dec);
            return got < 0 ? got : OPUS_INTERNAL_ERROR;
         }
         total += got;
         opus_decoder_ctl(dec, OPUS_GET_FINAL_RANGE(&decoded_range));
         if (decoded_range != packet->range)
            (*range_errors)++;
      }
   }
   if (pending) {
      while (pending > 0) {
         int got = opus_decode(dec, NULL, 0,
                               &run->decoded[(size_t)total*run->channels],
                               frame_samples, 0);
         if (got <= 0) {
            opus_decoder_destroy(dec);
            return got < 0 ? got : OPUS_INTERNAL_ERROR;
         }
         total += got;
         (*plc_frames)++;
         pending--;
      }
   }
   opus_decoder_destroy(dec);
   run->decoded_frames = total;
   return OPUS_OK;
}

static void cross_test_loss_recovery(CrossContext *ctx,
                                     const CrossConfig *cfg)
{
   CrossRun fec;
   CrossRun plc;
   CrossRun clean;
   CrossMetrics phase;
   CrossMetrics recovery;
   CrossSegment segment;
   char scenario[96];
   int frames = 2*48000;
   int fec_recovered = 0;
   int unused_fec = 0;
   int fec_plc = 0;
   int plc_plc = 0;
   int fec_range_errors = 0;
   int plc_range_errors = 0;
   int rc;
   int loss_ms[16];
   int loss_count = 0;
   int i;
   snprintf(scenario, sizeof(scenario), "fec-plc br=%d cx=%d",
            cfg->bitrate, cfg->complexity);
   memset(&segment, 0, sizeof(segment));
   segment.bitrate = cfg->bitrate < 32000 ? 32000 : cfg->bitrate;
   segment.complexity = cfg->complexity;
   segment.signal = OPUS_SIGNAL_VOICE;
   segment.vbr = 1;
   segment.fec = 1;
   segment.loss = 20;
   cross_run_init(&fec, 48000, 2, frames, CROSS_PCM_SPEECH, ctx->seed+5);
   cross_run_init(&plc, 48000, 2, frames, CROSS_PCM_SPEECH, ctx->seed+5);
   cross_run_init(&clean, 48000, 2, frames, CROSS_PCM_SPEECH, ctx->seed+5);
   rc = cross_encode_run(&fec, OPUS_APPLICATION_VOIP, 48000/50,
                         &segment, 1);
   segment.fec = 0;
   segment.loss = 0;
   if (rc == OPUS_OK)
      rc = cross_encode_run(&plc, OPUS_APPLICATION_VOIP, 48000/50,
                            &segment, 1);
   if (rc == OPUS_OK)
      rc = cross_encode_run(&clean, OPUS_APPLICATION_VOIP, 48000/50,
                            &segment, 1);
   cross_assert(ctx, scenario, "encode", rc == OPUS_OK, "%s",
                opus_strerror(rc));
   if (rc == OPUS_OK) {
      for (i = 0; i < fec.packet_count; ++i) {
         if (i % 13 == 5) {
            loss_ms[loss_count++] = i*20;
         }
      }
      rc = cross_decode_with_loss(&fec, 1, &fec_recovered, &fec_plc,
                                  &fec_range_errors);
      if (rc == OPUS_OK)
         rc = cross_decode_with_loss(&plc, 0, &unused_fec, &plc_plc,
                                     &plc_range_errors);
      if (rc == OPUS_OK)
         rc = cross_decode_run(&clean);
   }
   cross_assert(ctx, scenario, "decode", rc == OPUS_OK, "%s",
                opus_strerror(rc));
   cross_report_run(ctx, scenario, &fec, frames);
   cross_report_run(ctx, scenario, &plc, frames);
   cross_assert(ctx, scenario, "fec-used", fec_recovered > 0,
                "recovered-by-fec=%d", fec_recovered);
   cross_assert(ctx, scenario, "plc-used",
                plc_plc == loss_count && fec_plc + fec_recovered == loss_count,
                "plc=%d fec=%d fec-plc=%d losses=%d",
                plc_plc, fec_recovered, fec_plc, loss_count);
   cross_assert(ctx, scenario, "fec-range-recovery", fec_range_errors == 0,
                "errors=%d packets=%d", fec_range_errors,
                fec.packet_count);
   cross_assert(ctx, scenario, "plc-range-recovery", plc_range_errors == 0,
                "errors=%d packets=%d", plc_range_errors,
                plc.packet_count);
   cross_measure_run(&clean, 48000/50, &phase);
   cross_check_phase(ctx, scenario, &phase);
   cross_assert(ctx, scenario, "loss-energy-jump",
                phase.max_jump_db <= 30.0,
                "max=%.3f dB limit=30.000 dB", phase.max_jump_db);
   cross_compare_runs(&fec, &clean, 48000/50, loss_ms, loss_count, 200,
                      &recovery);
   cross_check_lossy_custom(ctx, scenario, &recovery, 4.0, 0.70, 65.0);
   cross_run_clear(&fec);
   cross_run_clear(&plc);
   cross_run_clear(&clean);
}

static void cross_test_state_isolation(CrossContext *ctx,
                                       const CrossConfig *cfg)
{
   CrossRun warm_packets;
   CrossRun target_reset;
   CrossRun target_fresh;
   CrossSegment warm_segments[3];
   CrossSegment target_segment;
   OpusEncoder *warm;
   OpusEncoder *fresh;
   int err;
   int force_channels = 0;
   int i;
   int equal = 1;
   char scenario[96];
   snprintf(scenario, sizeof(scenario), "state-isolation br=%d cx=%d",
            cfg->bitrate, cfg->complexity);
   memset(warm_segments, 0, sizeof(warm_segments));
   warm_segments[0].start_ms = 0;
   warm_segments[0].bitrate = 64000;
   warm_segments[0].complexity = cfg->complexity;
   warm_segments[0].signal = OPUS_SIGNAL_MUSIC;
   warm_segments[0].vbr = 1;
   warm_segments[1].start_ms = 200;
   warm_segments[1].bitrate = 10000;
   warm_segments[1].complexity = cfg->complexity;
   warm_segments[1].signal = OPUS_SIGNAL_VOICE;
   warm_segments[1].vbr = 1;
   warm_segments[2].start_ms = 400;
   warm_segments[2].bitrate = 64000;
   warm_segments[2].complexity = cfg->complexity;
   warm_segments[2].signal = OPUS_SIGNAL_MUSIC;
   warm_segments[2].vbr = 1;
   memset(&target_segment, 0, sizeof(target_segment));
   target_segment.bitrate = cfg->bitrate;
   target_segment.complexity = cfg->complexity;
   target_segment.signal = OPUS_SIGNAL_VOICE;
   target_segment.vbr = 1;
   cross_run_init(&warm_packets, 48000, 2, 48000*3/5,
                  CROSS_PCM_MIXED, ctx->seed+6);
   cross_run_init(&target_reset, 48000, 2, 48000,
                  CROSS_PCM_SPEECH, ctx->seed+7);
   cross_run_init(&target_fresh, 48000, 2, 48000,
                  CROSS_PCM_SPEECH, ctx->seed+7);
   warm = opus_encoder_create(48000, 2, OPUS_APPLICATION_VOIP, &err);
   fresh = opus_encoder_create(48000, 2, OPUS_APPLICATION_VOIP, &err);
   cross_assert(ctx, scenario, "encoder-create",
                warm != NULL && fresh != NULL, "warm/fresh allocation");
   if (warm != NULL && fresh != NULL) {
      int rc = cross_encode_with_encoder(&warm_packets, warm, 48000/10,
                                         warm_segments, 3);
      cross_assert(ctx, scenario, "warm-encode", rc == OPUS_OK, "%s",
                   opus_strerror(rc));
      opus_encoder_ctl(warm, OPUS_GET_FORCE_CHANNELS(&force_channels));
      cross_assert(ctx, scenario, "force-channel-restored",
                   force_channels == OPUS_AUTO,
                   "got=%d expected=%d", force_channels, OPUS_AUTO);
      opus_encoder_ctl(warm, OPUS_RESET_STATE);
      rc = cross_encode_with_encoder(&target_reset, warm, 48000/50,
                                     &target_segment, 1);
      if (rc == OPUS_OK)
         rc = cross_encode_with_encoder(&target_fresh, fresh, 48000/50,
                                        &target_segment, 1);
      cross_assert(ctx, scenario, "reset-encode", rc == OPUS_OK, "%s",
                   opus_strerror(rc));
      if (rc == OPUS_OK) {
         equal = target_reset.packet_count == target_fresh.packet_count
              && target_reset.total_bytes == target_fresh.total_bytes;
         if (equal) {
            for (i = 0; i < target_reset.packet_count; ++i) {
               CrossPacket *a = &target_reset.packets[i];
               CrossPacket *b = &target_fresh.packets[i];
               if (a->len != b->len
                   || memcmp(a->data, b->data, a->len) != 0) {
                  equal = 0;
                  break;
               }
            }
         }
      } else {
         equal = 0;
      }
      cross_assert(ctx, scenario, "reset-byte-identical", equal,
                   "rst=%d/%d fresh=%d/%d",
                   target_reset.packet_count, target_reset.total_bytes,
                   target_fresh.packet_count, target_fresh.total_bytes);
   }
   if (warm != NULL) opus_encoder_destroy(warm);
   if (fresh != NULL) opus_encoder_destroy(fresh);
   cross_run_clear(&warm_packets);
   cross_run_clear(&target_reset);
   cross_run_clear(&target_fresh);
}

static void cross_test_decoder_reset(CrossContext *ctx,
                                     const CrossConfig *cfg)
{
   CrossRun packets;
   CrossSegment segment;
   OpusDecoder *warm;
   OpusDecoder *fresh;
   short *warm_out;
   short *reset_out;
   short *fresh_out;
   int err;
   int warm_frames = 0;
   int reset_frames = 0;
   int fresh_frames = 0;
   int encode_rc;
   int i;
   int same = 1;
   char scenario[96];
   snprintf(scenario, sizeof(scenario), "decoder-reset br=%d cx=%d",
            cfg->bitrate, cfg->complexity);
   memset(&segment, 0, sizeof(segment));
   segment.bitrate = cfg->bitrate;
   segment.complexity = cfg->complexity;
   segment.signal = OPUS_SIGNAL_VOICE;
   segment.vbr = 1;
   cross_run_init(&packets, 48000, 2, 48000, CROSS_PCM_SPEECH,
                  ctx->seed+8);
   encode_rc = cross_encode_run(&packets, OPUS_APPLICATION_VOIP, 48000/50,
                                &segment, 1);
   cross_assert(ctx, scenario, "encode", encode_rc == OPUS_OK, "%s",
                opus_strerror(encode_rc));
   warm_out = (short *)calloc((size_t)(packets.pcm_frames+48000/5)*2,
                              sizeof(*warm_out));
   reset_out = (short *)calloc((size_t)(packets.pcm_frames+48000/5)*2,
                               sizeof(*reset_out));
   fresh_out = (short *)calloc((size_t)(packets.pcm_frames+48000/5)*2,
                               sizeof(*fresh_out));
   warm = opus_decoder_create(48000, 2, &err);
   fresh = opus_decoder_create(48000, 2, &err);
   if (warm_out == NULL || reset_out == NULL || fresh_out == NULL) {
      fprintf(stderr, "out of memory\n");
      exit(1);
   }
   cross_assert(ctx, scenario, "decoder-create",
                warm != NULL && fresh != NULL, "warm/fresh allocation");
   if (warm != NULL && fresh != NULL) {
      int rc = encode_rc;
      for (i = 0; i < packets.packet_count; ++i) {
         int got = opus_decode(warm, packets.packets[i].data,
                               packets.packets[i].len,
                               &warm_out[(size_t)warm_frames*2],
                               48000/5, 0);
         if (got <= 0) {
            rc = got;
            break;
         }
         warm_frames += got;
      }
      opus_decoder_ctl(warm, OPUS_RESET_STATE);
      if (rc == OPUS_OK) {
         for (i = 0; i < packets.packet_count; ++i) {
            int got = opus_decode(warm, packets.packets[i].data,
                                  packets.packets[i].len,
                                  &reset_out[(size_t)reset_frames*2],
                                  48000/5, 0);
            if (got <= 0) {
               rc = got;
               break;
            }
            reset_frames += got;
         }
      }
      if (rc == OPUS_OK) {
         for (i = 0; i < packets.packet_count; ++i) {
            int got = opus_decode(fresh, packets.packets[i].data,
                                  packets.packets[i].len,
                                  &fresh_out[(size_t)fresh_frames*2],
                                  48000/5, 0);
            if (got <= 0) {
               rc = got;
               break;
            }
            fresh_frames += got;
         }
      }
      cross_assert(ctx, scenario, "decode", rc == OPUS_OK, "%s",
                   opus_strerror(rc));
      if (rc != OPUS_OK || reset_frames != fresh_frames)
         same = 0;
      if (same && memcmp(reset_out, fresh_out,
                         (size_t)reset_frames*2*sizeof(*reset_out)) != 0)
         same = 0;
      cross_assert(ctx, scenario, "reset-sample-identical", same,
                   "warm=%d reset=%d fresh=%d",
                   warm_frames, reset_frames, fresh_frames);
   }
   if (warm != NULL) opus_decoder_destroy(warm);
   if (fresh != NULL) opus_decoder_destroy(fresh);
   free(warm_out);
   free(reset_out);
   free(fresh_out);
   cross_run_clear(&packets);
}

static void cross_test_plc_cap(CrossContext *ctx)
{
   OpusDecoder *dec;
   short *out;
   int err;
   int got;
   const char *scenario = "plc-duration-cap";
   dec = opus_decoder_create(48000, 2, &err);
   out = (short *)calloc((size_t)48000*4, sizeof(*out));
   if (out == NULL) {
      fprintf(stderr, "out of memory\n");
      exit(1);
   }
   cross_assert(ctx, scenario, "decoder-create",
                dec != NULL && err == OPUS_OK, "err=%d", err);
   if (dec != NULL) {
      got = opus_decode(dec, NULL, 0, out, 48000*2, 0);
      cross_assert(ctx, scenario, "one-second-cap", got == 48000,
                   "decoded=%d limit=%d", got, 48000);
   }
   if (dec != NULL) opus_decoder_destroy(dec);
   free(out);
}

static void cross_usage(const char *program)
{
   printf("Usage: %s [options]\n", program);
   printf("  --quick              run the default 3-rate, 2-complexity matrix\n");
   printf("  --full               run 9 rates and complexity 0, 4 and 10\n");
   printf("  --bitrate=N[,N...]   override the bitrate matrix\n");
   printf("  --complexity=N[,N]   override the complexity matrix\n");
   printf("  --seed=N             deterministic PCM seed (default 1)\n");
   printf("  --verbose            print passing checks\n");
   printf("  --json               emit JSON status lines\n");
   printf("  --fail-fast          stop at the first failed check\n");
   printf("  --list               list scenarios and fixed thresholds\n");
   printf("  --help               show this text\n");
}

static int cross_parse_values(const char *text, int *values, int capacity,
                              int min_value, int max_value)
{
   int count = 0;
   const char *p = text;
   while (*p != '\0') {
      char *end;
      long v;
      if (count >= capacity)
         return -1;
      v = strtol(p, &end, 10);
      if (end == p || v < min_value || v > max_value)
         return -1;
      values[count++] = (int)v;
      if (*end == '\0')
         break;
      if (*end != ',')
         return -1;
      p = end + 1;
   }
   return count;
}

static void cross_list_scenarios(void)
{
   printf("Scenarios:\n");
   printf("  steady-cbr           fixed CBR packet sizes, 20 ms vs 100 ms\n");
   printf("  chunk-20-vs-100      same duration, different API chunking\n");
   printf("  celt-2.5-vs-5ms      CELT ultra-short frame boundaries\n");
   printf("  dtx                  silent interval and comfort-noise recovery\n");
   printf("  fec-plc              loss recovery, FEC and PLC paths\n");
   printf("  state-isolation      force-channel state and reset replay\n");
   printf("  decoder-reset        decoder reset replay\n");
   printf("  plc-duration-cap     one-second FEC/PLC request bound\n");
   printf("\nThresholds:\n");
   printf("  state replay: bytes and samples must match exactly\n");
   printf("  sample count: decoded samples must equal input duration\n");
   printf("  FEC/PLC range: final range must match the encoder exactly\n");
   printf("  CBR: packet size must be constant and totals within 64 bytes\n");
   printf("  chunking energy: <= %.1f dB in settled regions\n",
          CROSS_MAX_ENERGY_DELTA_DB);
   printf("  chunking RMS delta: <= %.1f%% in settled regions\n",
          CROSS_MAX_SIGNAL_RMS_PCT);
   printf("  waveform correlation: >= %.2f in settled regions\n",
          CROSS_MIN_SIGNAL_CORR);
   printf("  stereo phase: >= %.2f (polarity check)\n",
          CROSS_MIN_LR_PHASE);
   printf("  active energy jump: <= %.1f dB\n", CROSS_MAX_JUMP_DB);
   printf("  CBR below 16 kb/s: waveform delta skipped, jump <= 30.0 dB\n");
   printf("  DTX: silence onset <= 36.0 dB, quiet <= 25%% of active RMS\n");
   printf("  FEC/PLC: 4.0 dB, correlation 0.70, RMS delta 65%% after 200 ms\n");
   printf("  2.5 ms CELT: 3.0 dB, correlation 0.70, RMS delta 80%%\n");
}

int main(int argc, char **argv)
{
   CrossContext ctx;
   CrossConfig cfg;
   int rates[32];
   int complexities[32];
   int rate_count;
   int complexity_count;
   int i;
   int j;
   int list_only = 0;
   memset(&ctx, 0, sizeof(ctx));
   ctx.seed = 1;
   rate_count = (int)(sizeof(cross_short_rates)/sizeof(cross_short_rates[0]));
   memcpy(rates, cross_short_rates, sizeof(cross_short_rates));
   complexities[0] = 4;
   complexities[1] = 10;
   complexity_count = 2;
   for (i = 1; i < argc; ++i) {
      const char *arg = argv[i];
      if (strcmp(arg, "--quick") == 0) {
         rate_count = (int)(sizeof(cross_short_rates)
                           /sizeof(cross_short_rates[0]));
         memcpy(rates, cross_short_rates, sizeof(cross_short_rates));
         complexities[0] = 4;
         complexities[1] = 10;
         complexity_count = 2;
      } else if (strcmp(arg, "--full") == 0) {
         rate_count = (int)(sizeof(cross_full_rates)
                           /sizeof(cross_full_rates[0]));
         memcpy(rates, cross_full_rates, sizeof(cross_full_rates));
         complexities[0] = 0;
         complexities[1] = 4;
         complexities[2] = 10;
         complexity_count = 3;
      } else if (strcmp(arg, "--verbose") == 0) {
         ctx.verbose = 1;
      } else if (strcmp(arg, "--json") == 0) {
         ctx.json = 1;
      } else if (strcmp(arg, "--fail-fast") == 0) {
         ctx.fail_fast = 1;
      } else if (strcmp(arg, "--list") == 0) {
         list_only = 1;
      } else if (strcmp(arg, "--help") == 0 || strcmp(arg, "-h") == 0) {
         cross_usage(argv[0]);
         return 0;
      } else if (strncmp(arg, "--bitrate=", 10) == 0) {
         rate_count = cross_parse_values(arg + 10, rates, 32, 500, 512000);
         if (rate_count <= 0) {
            fprintf(stderr, "invalid --bitrate value\n");
            return 2;
         }
      } else if (strncmp(arg, "--complexity=", 13) == 0) {
         complexity_count = cross_parse_values(arg + 13, complexities, 32,
                                               0, 10);
         if (complexity_count <= 0) {
            fprintf(stderr, "invalid --complexity value\n");
            return 2;
         }
      } else if (strncmp(arg, "--seed=", 7) == 0) {
         char *end;
         unsigned long seed = strtoul(arg + 7, &end, 10);
         if (*end != '\0') {
            fprintf(stderr, "invalid --seed value\n");
            return 2;
         }
         ctx.seed = (unsigned)seed;
      } else {
         fprintf(stderr, "unknown option: %s\n", arg);
         cross_usage(argv[0]);
         return 2;
      }
   }
   if (list_only) {
      cross_list_scenarios();
      return 0;
   }
   for (i = 0; i < rate_count; ++i) {
      for (j = 0; j < complexity_count; ++j) {
         cfg.bitrate = rates[i];
         cfg.complexity = complexities[j];
         cfg.vbr = 1;
         cross_test_steady_cbr(&ctx, &cfg);
         cross_test_chunking_switch(&ctx, &cfg);
         cross_test_short_celt(&ctx, &cfg);
         cross_test_dtx(&ctx, &cfg);
         cross_test_loss_recovery(&ctx, &cfg);
         cross_test_state_isolation(&ctx, &cfg);
         cross_test_decoder_reset(&ctx, &cfg);
         if (ctx.fail_fast && ctx.failures)
            break;
      }
      if (ctx.fail_fast && ctx.failures)
         break;
   }
   cross_test_plc_cap(&ctx);
   if (ctx.json) {
      printf("{\"status\":\"%s\",\"checks\":%ld,\"failures\":%ld,"
             "\"configs\":%d,\"seed\":%u}\n",
             ctx.failures ? "fail" : "pass", ctx.checks, ctx.failures,
             rate_count*complexity_count, ctx.seed);
   } else {
      printf("Opus cross-frame consistency: %ld checks, %ld failures, "
             "%d configurations, seed %u\n",
             ctx.checks, ctx.failures, rate_count*complexity_count,
             ctx.seed);
   }
   return ctx.failures ? 1 : 0;
}

