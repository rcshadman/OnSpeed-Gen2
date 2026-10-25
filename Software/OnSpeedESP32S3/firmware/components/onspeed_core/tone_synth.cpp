#include "tone_synth.h"
#include <math.h>

namespace onspeed {

static float s_sine[1024];
static bool s_sineInit = false;

ToneSynth::ToneSynth(int sampleRate) : rate_(sampleRate) {
  if (!s_sineInit) {
    for (int i = 0; i < 1024; i++) s_sine[i] = sinf(2 * (float)M_PI * i / 1024);
    s_sineInit = true;
  }
}

void ToneSynth::setToneState(ToneMode mode, bool highTone, float pps) {
  toneMode_ = mode;
  highTone_ = highTone;
  pps_ = pps;
}

void ToneSynth::update3DAudio(float lateralG) {
  const float smoothingFactor = 0.1f;
  int sign = (lateralG > 0) - (lateralG < 0);
  float x = fabsf(lateralG);
  float curveGain = -92.822f * x * x + 20.025f * x;  // AUDIO_3D_CURVE
  if (curveGain > 1) curveGain = 1;
  if (curveGain < 0) curveGain = 0;
  curveGain *= sign;
  channelGain_ = smoothingFactor * curveGain + (1 - smoothingFactor) * channelGain_;
  gainL_ = fabsf(-1 + channelGain_);
  gainR_ = fabsf(1 + channelGain_);
}

void ToneSynth::noteOn(float delayMs, float attackMs, float holdMs, float decayMs, float sustain, float releaseMs) {
  dly_ = ms2s(delayMs); att_ = ms2s(attackMs); hold_ = ms2s(holdMs);
  dec_ = ms2s(decayMs); sus_ = sustain; rel_ = ms2s(releaseMs);
  env_ = ENV_DELAY; envCount_ = dly_;
  envLevel_ = 0;  // releaseNoteOn(0): previous note is cut immediately
}

void ToneSynth::noteOff() {
  if (env_ == ENV_IDLE) return;
  env_ = ENV_RELEASE;
  envCount_ = rel_;
  envStep_ = rel_ > 0 ? -envLevel_ / rel_ : 0;
  if (rel_ == 0) { envLevel_ = 0; env_ = ENV_IDLE; }
}

void ToneSynth::tonePlayHandler() {
  if (toneMode_ == TONE_OFF) { setFrequencytone(0); return; }
  if (toneMode_ == SOLID_TONE) { setFrequencytone(LOW_TONE_HZ); return; }
  setFrequencytone(highTone_ ? HIGH_TONE_HZ : LOW_TONE_HZ);
}

void ToneSynth::setFrequencytone(uint32_t frequency) {
  float pulse_delay = 1000 / pps_;
  float tone_length = pulse_delay - 3;
  // re-arm the tone timer (IntervalTimer ToneTimer in Gen2)
  if (highTone_ && tonePlaying_ == SOLID_TONE)
    nextHandler_ = sampleIndex_ + ms2s(pulse_delay / 2 + 1000 / LOW_TONE_PPS_MAX / 2);
  else
    nextHandler_ = sampleIndex_ + ms2s(pulse_delay);

  if (frequency < 20 || frequency > 20000 || toneMode_ == TONE_OFF) {
    amplitude_ = 0;
    toneFreq_ = frequency;
    tonePlaying_ = TONE_OFF;
    return;
  }
  float volume;
  if (toneMode_ == SOLID_TONE) {
    volume = SOLID_TONE_VOLUME;
    if (tonePlaying_ != SOLID_TONE) {
      noteOff();
      phaseInc_ = (uint32_t)((double)frequency * 4294967296.0 / rate_);
      amplitude_ = volume;
      noteOn(1000 / LOW_TONE_PPS_MAX / 2, TONE_RAMP_TIME, 0, 0, 1, TONE_RAMP_TIME);
      tonePlaying_ = SOLID_TONE;
      if ((evHead_ + 1) % 32 != evTail_) { ev_[evHead_] = {sampleIndex_, (uint16_t)frequency, SOLID_TONE, volume}; evHead_ = (evHead_ + 1) % 32; }
    }
  } else {
    phaseInc_ = (uint32_t)((double)frequency * 4294967296.0 / rate_);
    if (!highTone_) volume = LOW_TONE_VOLUME;
    else if (pps_ == HIGH_TONE_STALL_PPS) volume = HIGH_TONE_VOLUME_MAX;
    else {
      volume = mapfloat(pps_, HIGH_TONE_PPS_MIN, HIGH_TONE_PPS_MAX, HIGH_TONE_VOLUME_MIN, HIGH_TONE_VOLUME_MAX);
      if (volume < HIGH_TONE_VOLUME_MIN) volume = HIGH_TONE_VOLUME_MIN;
      if (volume > HIGH_TONE_VOLUME_MAX) volume = HIGH_TONE_VOLUME_MAX;
    }
    noteOff();
    amplitude_ = volume;
    float dly = (highTone_ && tonePlaying_ == SOLID_TONE) ? (1000 / LOW_TONE_PPS_MAX) / 2 : tone_length / 2;
    float ramp = (highTone_ && pps_ == HIGH_TONE_STALL_PPS) ? STALL_RAMP_TIME : TONE_RAMP_TIME;
    noteOn(dly, ramp, tone_length / 2 - ramp * 2, ramp, 0, 0);
    tonePlaying_ = PULSE_TONE;
    if ((evHead_ + 1) % 32 != evTail_) { ev_[evHead_] = {sampleIndex_ + (uint32_t)ms2s(dly), (uint16_t)frequency, PULSE_TONE, volume}; evHead_ = (evHead_ + 1) % 32; }
  }
  toneFreq_ = frequency;
}

bool ToneSynth::popEvent(ToneEvent &e) {
  if (evTail_ == evHead_) return false;
  e = ev_[evTail_];
  evTail_ = (evTail_ + 1) % 32;
  return true;
}

void ToneSynth::render(int16_t *out, int n) {
  for (int i = 0; i < n; i++) {
    if ((int64_t)sampleIndex_ >= nextHandler_) tonePlayHandler();
    // envelope state machine (AudioEffectEnvelope)
    switch (env_) {
      case ENV_DELAY:
        if (envCount_-- <= 0) { env_ = ENV_ATTACK; envCount_ = att_; envStep_ = att_ > 0 ? 1.0f / att_ : 1; }
        break;
      case ENV_ATTACK:
        envLevel_ += envStep_;
        if (--envCount_ <= 0) { envLevel_ = 1; env_ = ENV_HOLD; envCount_ = hold_; }
        break;
      case ENV_HOLD:
        if (envCount_-- <= 0) { env_ = ENV_DECAY; envCount_ = dec_; envStep_ = dec_ > 0 ? (sus_ - 1) / dec_ : 0; if (dec_ == 0) envLevel_ = sus_; }
        break;
      case ENV_DECAY:
        envLevel_ += envStep_;
        if (--envCount_ <= 0) { envLevel_ = sus_; env_ = ENV_SUSTAIN; }
        break;
      case ENV_RELEASE:
        envLevel_ += envStep_;
        if (--envCount_ <= 0) { envLevel_ = 0; env_ = ENV_IDLE; }
        break;
      default: break;
    }
    if (envLevel_ < 0) envLevel_ = 0;
    float s = s_sine[phase_ >> 22] * amplitude_ * envLevel_ * master_;
    phase_ += phaseInc_;
    float l = s * gainL_ * 0.5f, r = s * gainR_ * 0.5f;  // gains reach 2.0 at full pan
    out[2 * i] = (int16_t)(l * 32767);
    out[2 * i + 1] = (int16_t)(r * 32767);
    sampleIndex_++;
  }
}

}  // namespace onspeed
