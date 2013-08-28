#include "vision/JPEGRoutines.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <errno.h>
#include <string.h>

#include "aos/common/time.h"

#include "vision/OpenCVWorkTask.h"

namespace frc971 {
namespace vision {

/* This is also adapted from libjpeg to be used on decompression tables rather than
 * compression tables as it was origionally intended
 */
void decompress_add_huff_table (j_decompress_ptr cinfo,
    JHUFF_TBL **htblptr, const UINT8 *bits, const UINT8 *val)
/* Define a Huffman table */
{
  int nsymbols, len;

  if (*htblptr == NULL)
    *htblptr = jpeg_alloc_huff_table((j_common_ptr) cinfo);

  /* Copy the number-of-symbols-of-each-code-length counts */
  memcpy((*htblptr)->bits, bits, sizeof((*htblptr)->bits));

  /* Validate the counts.  We do this here mainly so we can copy the right
   * number of symbols from the val[] array, without risking marching off
   * the end of memory.  jchuff.c will do a more thorough test later.
   */
  nsymbols = 0;
  for (len = 1; len <= 16; len++)
    nsymbols += bits[len];
  if (nsymbols < 1 || nsymbols > 256){
    fprintf(stderr,"%s:%d: Error, bad huffman table",__FILE__,__LINE__);
    exit(-1);
  }

  memcpy((*htblptr)->huffval, val, nsymbols * sizeof(uint8_t));

}

/* standard_huff_tables is taken from libjpeg compression stuff
 * and is here used to set up the same tables in the decompression structure.
 */
void  standard_huff_tables (j_decompress_ptr cinfo)
  /* Set up the standard Huffman tables (cf. JPEG standard section K.3) */
  /* IMPORTANT: these are only valid for 8-bit data precision! */
{
  static const UINT8 bits_dc_luminance[17] =
  { /* 0-base */ 0, 0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0 };
  static const UINT8 val_dc_luminance[] =
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

  static const UINT8 bits_dc_chrominance[17] =
  { /* 0-base */ 0, 0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 };
  static const UINT8 val_dc_chrominance[] =
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };

  static const UINT8 bits_ac_luminance[17] =
  { /* 0-base */ 0, 0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d };
  static const UINT8 val_ac_luminance[] =
  { 0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
    0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
    0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
    0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
    0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
    0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
    0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
    0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
    0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
    0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
    0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
    0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
    0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
    0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
    0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
    0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
    0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
    0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
    0xf9, 0xfa };

  static const UINT8 bits_ac_chrominance[17] =
  { /* 0-base */ 0, 0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77 };
  static const UINT8 val_ac_chrominance[] =
  { 0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
    0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71,
    0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
    0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
    0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34,
    0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
    0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
    0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48,
    0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
    0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
    0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78,
    0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
    0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
    0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5,
    0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
    0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
    0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
    0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
    0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
    0xf9, 0xfa };

  decompress_add_huff_table(cinfo, &cinfo->dc_huff_tbl_ptrs[0],
      bits_dc_luminance, val_dc_luminance);
  decompress_add_huff_table(cinfo, &cinfo->ac_huff_tbl_ptrs[0],
      bits_ac_luminance, val_ac_luminance);
  decompress_add_huff_table(cinfo, &cinfo->dc_huff_tbl_ptrs[1],
      bits_dc_chrominance, val_dc_chrominance);
  decompress_add_huff_table(cinfo, &cinfo->ac_huff_tbl_ptrs[1],
      bits_ac_chrominance, val_ac_chrominance);
}




void process_jpeg(unsigned char *out,unsigned char *image,size_t size){
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;

  static aos::time::Time timestamp_old = aos::time::Time::Now();
  //aos::time::Time timestamp_start = aos::time::Time::Now();
  
  cinfo.err = jpeg_std_error( &jerr );
  cinfo.out_color_space = JCS_RGB;
  jpeg_create_decompress( &cinfo );
  //jpeg_stdio_src( &cinfo, infile );
  jpeg_mem_src(&cinfo,image,size);

  jpeg_read_header( &cinfo, TRUE );
  standard_huff_tables (&cinfo);


//  printf( "JPEG File Information: \n" );
//  printf( "Image width and height: %d pixels and %d pixels.\n", cinfo.image_width, cinfo.image_height );
//  printf( "Color components per pixel: %d.\n", cinfo.num_components );
//  printf( "Color space: %d.\n", cinfo.jpeg_color_space );
  //printf("JpegDecompressed\n");

  jpeg_start_decompress( &cinfo );

  int offset = 0;
  int step = cinfo.num_components * cinfo.image_width;
  unsigned char *buffers[cinfo.image_height];
  for (int i = cinfo.image_height - 1; i >= 0; --i) {
    buffers[i] = &out[offset]; 
    offset += step;
  }

  while( cinfo.output_scanline < cinfo.image_height )
  {
    jpeg_read_scanlines(&cinfo, &buffers[cinfo.output_scanline],
        cinfo.image_height - cinfo.output_scanline);
  }

  /* This used to do BGR to RGB conversions inline */
/* 
  for (int i = 0; i < (int)(cinfo.image_height * cinfo.image_width * 3); i+= 3) {
    uint8_t b = out[i + 0];
    uint8_t r = out[i + 2];
    out[i + 0] = r;
    out[i + 2] = b;
  }
*/
  jpeg_finish_decompress( &cinfo );
  jpeg_destroy_decompress( &cinfo );

  aos::time::Time timestamp_end = aos::time::Time::Now();

  //double jpeg_part = ((timestamp_end - timestamp_start).nsec()) / 1000000000.0;
  //double longer_part = ((timestamp_end - timestamp_old).nsec()) / 1000000000.0;

  //printf("%g %g\n",jpeg_part / longer_part,1.0 / longer_part);

  timestamp_old = timestamp_end;

}

}  // namespace vision
}  // namespace frc971

