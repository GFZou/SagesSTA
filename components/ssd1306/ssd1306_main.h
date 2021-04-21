#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Register system functions
void * ssd1306_main(void * p);

void * ssd1306_showTemp(void * p);

void * ssd1306_showinfo(void * p);
void settemp(int h,int l);
void sethumi(int h,int l);
void settovc(float p);
void setnetinfo(int n,int s);

#ifdef __cplusplus
}
#endif
