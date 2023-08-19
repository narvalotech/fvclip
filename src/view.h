#ifndef VIEW_H_
#define VIEW_H_

struct viewstate {
	int8_t program;
	uint8_t active;
	uint8_t values[3];
	char names[3][8];
};

void draw_view(struct viewstate *state);
void view_init(void);

#endif // VIEW_H_
