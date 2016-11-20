
/*********************************************************/
/* UCFReadPic.c                                          */
/*********************************************************/
PIC UCFReadPic(infile)
FILE *infile;
{
   PIC temp;
   unsigned char byte1,byte2,byte3,byte4;

   /* getting Type from image data */
/*   fread(&temp.type,sizeof(temp.type),1,infile); */
   fread(&byte1,sizeof(BYTE),1,infile);
   fread(&byte2,sizeof(BYTE),1,infile);
   fread(&byte3,sizeof(BYTE),1,infile);
   fread(&byte4,sizeof(BYTE),1,infile);
   temp.type= byte1*16777216+byte2*65536+byte3*256+byte4;
   switch (temp.type)
   {
      case 0xF10F:
      case 0xF200:
      case 0xF201:
      case 0xF204:
      case 0x0000:
      {  
/*         fread(&temp.maxX,sizeof(temp.maxX),1,infile); */
/*         fread(&temp.maxY,sizeof(temp.maxY),1,infile); */
        fread(&byte1,sizeof(BYTE),1,infile);
        fread(&byte2,sizeof(BYTE),1,infile);
        fread(&byte3,sizeof(BYTE),1,infile);
        fread(&byte4,sizeof(BYTE),1,infile);
        temp.maxX= byte1*16777216+byte2*65536+byte3*256+byte4;
        fread(&byte1,sizeof(BYTE),1,infile);
        fread(&byte2,sizeof(BYTE),1,infile);
        fread(&byte3,sizeof(BYTE),1,infile);
        fread(&byte4,sizeof(BYTE),1,infile);
        temp.maxY= byte1*16777216+byte2*65536+byte3*256+byte4;
        break;
      }
      case 0x8000:
      case 0x8001:
      case 0xB003:
      default  :
      {
        printf("****** Unknow image type *****\n");
        exit(1);
      }
   }

   if((temp.image=(BYTE*)calloc(temp.maxX*temp.maxY,sizeof(BYTE)))==NULL)
   {
      printf("***** NOT enough memory *****\n");
      exit(1);
   }

   fread(temp.image,sizeof(BYTE),temp.maxX * temp.maxY,infile);
   return(temp);
}

