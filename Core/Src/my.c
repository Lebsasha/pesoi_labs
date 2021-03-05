#include <main.h>
#include <my.h>

void make_tone(Tone_pin* tone_pin)
{
    *tone_pin->duty_cycle = (uint32_t)(tone_pin->f_dots[tone_pin->curr>>8])*COUNTER_PERIOD/tone_pin->sine_ampl;
    tone_pin->curr+=tone_pin->dx;
    if (tone_pin->curr >= tone_pin->arr_size<<8)
    {
        tone_pin->curr-=tone_pin->arr_size<<8;
    }

}

void play(Tone_pin* pin, const uint16_t* notes, const uint8_t* durations, int n)
{
    volatile uint32_t wait;///TODO remove volatile
    uint32_t start_tick = HAL_GetTick();
    for (int i = 0; i<n; ++i)
    {
        pin->dx=(pin->arr_size<<8)*notes[i]/TONE_FREQ;
        wait=1000/durations[i];
        while ((HAL_GetTick() - start_tick) < wait)
        {
        }
        start_tick+=wait;
    }
}