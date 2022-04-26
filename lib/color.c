#include  <stdio.h>
#include  <stdlib.h>
 
int main(void)
{
 int color;
 int green, red, blue;
 
 /* 0 - 15 */
 puts("System colors:");
 for(color = 0; color<8; color++)
 {
   printf("\x1B[48;5;%dm  ", color);
 }
 puts("\x1B[0m");
 for(color = 8; color<16; color++)
 {
   printf("\x1B[48;5;%dm  ", color);
 }
 puts("\x1B[0m\n");
 
 /* 16 - 231 */
 puts("Color cube, 6x6x6:");
 for(red = 0; red<6; red++)
 {
   for(green = 0; green<6; green++)
   {
     for(blue = 0; blue<6; blue++)
     {
       color = 16 + (red * 36) + (green * 6) + blue;
       printf("\x1B[48;5;%dm  ", color);
     }
     printf("\x1B[0m ");
   }
   puts("");
 }
 
 /* 232 - 255 */
 puts("Grayscale ramp:");
 for (color = 232; color < 256; color++)
 {
   printf("\x1B[48;5;%dm  ", color);
 }
 puts("\x1B[0m");
 
   return 0;
}
