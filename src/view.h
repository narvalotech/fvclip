#ifndef VIEW_H_
#define VIEW_H_

struct viewstate {
	uint8_t program;
	uint8_t param;
	char *names[3];
	uint8_t values[3];
};

void draw_view(struct viewstate *state);
void view_init(void);

#endif // VIEW_H_
