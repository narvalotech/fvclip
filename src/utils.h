#ifndef UTILS_H_
#define UTILS_H_

#define PORT(n, p) DEVICE_DT_GET(DT_PHANDLE(DT_PATH(n, p), gpios))
#define PIN(n, p) DT_PHA(DT_PATH(n, p), gpios, pin)
#define PORT_PIN(n, p) PORT(n, p), PIN(n, p)

#endif // UTILS_H_
