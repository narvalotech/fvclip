#ifndef DSP_H_
#define DSP_H_

void init_dsp(void);
void load_from_serial(struct ring_buf *ringbuf, uint16_t len);
void set_pot_values(struct ring_buf *ringbuf, uint16_t len);

#endif // DSP_H_
