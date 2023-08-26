#ifndef DSP_H_
#define DSP_H_

void init_dsp(void);
void dsp_load_from_serial(struct ring_buf *ringbuf, uint16_t len);
void dsp_set_pot_values(struct ring_buf *ringbuf, uint16_t len);
void dsp_select_program(uint8_t id);
void dsp_set_pwm_channel(uint8_t ch, uint16_t val);

#endif // DSP_H_
