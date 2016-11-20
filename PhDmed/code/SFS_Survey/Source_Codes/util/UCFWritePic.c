/*************************************************************/
/* UCFWritePic.c  - Part of ImageTools.  See ImageTools.c    */
/*************************************************************/
void UCFWritePic (p,outfile)
PIC p;
FILE *outfile;
{
   unsigned char byte1, byte2, byte3, byte4;
   unsigned int temp,temp1;

   temp = p.type;
   byte4 = temp % 256;
   temp1 = temp - byte4;
   temp = (temp1/256);
   byte3 = temp % 256;
   temp1 = temp - byte3;
   temp = (temp1/256);
   byte2 = temp % 256;
   temp1 = temp - byte2;
   temp = (temp1/256);
   byte1 = temp % 256;
   temp1 = temp - byte1;
   fwrite(&byte1,sizeof(BYTE),1,outfile);
   fwrite(&byte2,sizeof(BYTE),1,outfile);
   fwrite(&byte3,sizeof(BYTE),1,outfile);
   fwrite(&byte4,sizeof(BYTE),1,outfile);
printf("%d %d %d %d\n",byte1, byte2, byte3, byte4);

   temp = p.maxX;
   byte4 = temp % 256;
   temp1 = temp - byte4;
   temp = (temp1/256);
   byte3 = temp % 256;
   temp1 = temp - byte3;
   temp = (temp1/256);
   byte2 = temp % 256;
   temp1 = temp - byte2;
   temp = (temp1/256);
   byte1 = temp % 256;
   temp1 = temp - byte1;
   fwrite(&byte1,sizeof(BYTE),1,outfile);
   fwrite(&byte2,sizeof(BYTE),1,outfile);
   fwrite(&byte3,sizeof(BYTE),1,outfile);
   fwrite(&byte4,sizeof(BYTE),1,outfile);
printf("%d %d %d %d\n",byte1, byte2, byte3, byte4);

   temp = p.maxY;
   byte4 = temp % 256;
   temp1 = temp - byte4;
   temp = (temp1/256);
   byte3 = temp % 256;
   temp1 = temp - byte3;
   temp = (temp1/256);
   byte2 = temp % 256;
   temp1 = temp - byte2;
   temp = (temp1/256);
   byte1 = temp % 256;
   temp1 = temp - byte1;
   fwrite(&byte1,sizeof(BYTE),1,outfile);
   fwrite(&byte2,sizeof(BYTE),1,outfile);
   fwrite(&byte3,sizeof(BYTE),1,outfile);
   fwrite(&byte4,sizeof(BYTE),1,outfile);
printf("%d %d %d %d\n",byte1, byte2, byte3, byte4);
/*
   fwrite(&p.type,sizeof(p.type),1,outfile);
   fwrite(&p.maxX,sizeof(p.maxX),1,outfile);
   fwrite(&p.maxY,sizeof(p.maxY),1,outfile);
*/

   fwrite(p.image,sizeof(unsigned char),(p.maxX * p.maxY),outfile);
   return;
}
