#include "aos/vision/image/jpeg_routines.h"

#include <errno.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>

#include "aos/common/logging/logging.h"
#include "third_party/libjpeg/jpeglib.h"

namespace aos {
namespace vision {

namespace {

void decompress_add_huff_table(j_decompress_ptr cinfo, JHUFF_TBL **htblptr,
                               const UINT8 *bits, const UINT8 *val);

void standard_huff_tables(j_decompress_ptr cinfo);

// Error handling form libjpeg
struct JpegErrorManager {
  /// "public" fields
  struct jpeg_error_mgr pub;
  // for return to caller
  jmp_buf setjmp_buffer;
};

char JpegLastErrorMsg[JMSG_LENGTH_MAX];

// TODO(parker): Error handling needs to be investigated bettter.
void JpegErrorExit(j_common_ptr cinfo) {
  JpegErrorManager myerr;
  // cinfo->err actually points to a JpegErrorManager struct
  ::std::memcpy(&myerr, cinfo->err, sizeof(myerr));
  // JpegErrorManager* myerr = (JpegErrorManager*) cinfo->err;
  // note : *(cinfo->err) is now equivalent to myerr->pub

  // output_message is a method to print an error message
  //(* (cinfo->err->output_message) ) (cinfo);

  // Create the message
  (*(cinfo->err->format_message))(cinfo, JpegLastErrorMsg);

  // Jump to the setjmp point
  longjmp(myerr.setjmp_buffer, 1);
}

// This is also adapted from libjpeg to be used on decompression tables rather
// than compression tables as it was originally intended.
void decompress_add_huff_table(j_decompress_ptr cinfo, JHUFF_TBL **htblptr,
                               const UINT8 *bits, const UINT8 *val) {
  if (*htblptr == NULL) *htblptr = jpeg_alloc_huff_table((j_common_ptr)cinfo);

  // Copy the number-of-symbols-of-each-code-length counts.
  memcpy((*htblptr)->bits, bits, sizeof((*htblptr)->bits));

  // Validate the counts.  We do this here mainly so we can copy the right
  // number of symbols from the val[] array, without risking marching off
  // the end of memory.  jchuff.c will do a more thorough test later.
  int nsymbols = 0;
  for (int len = 1; len <= 16; len++) nsymbols += bits[len];
  if (nsymbols < 1 || nsymbols > 256) {
    LOG(FATAL, "%s:%d: Error, bad huffman table", __FILE__, __LINE__);
  }

  memcpy((*htblptr)->huffval, val, nsymbols * sizeof(uint8_t));
}

// standard_huff_tables is taken from libjpeg compression stuff
// and is here used to set up the same tables in the decompression structure.
// Set up the standard Huffman tables (cf. JPEG standard section K.3)
// IMPORTANT: these are only valid for 8-bit data precision!
void standard_huff_tables(j_decompress_ptr cinfo) {
  /* 0-base on first 0, */
  static const UINT8 bits_dc_luminance[17] = {0, 0, 1, 5, 1, 1, 1, 1, 1,
                                              1, 0, 0, 0, 0, 0, 0, 0};
  static const UINT8 val_dc_luminance[] = {0, 1, 2, 3, 4,  5,
                                           6, 7, 8, 9, 10, 11};

  /* 0-base on first 0 */
  static const UINT8 bits_dc_chrominance[17] = {0, 0, 3, 1, 1, 1, 1, 1, 1,
                                                1, 1, 1, 0, 0, 0, 0, 0};
  static const UINT8 val_dc_chrominance[] = {0, 1, 2, 3, 4,  5,
                                             6, 7, 8, 9, 10, 11};

  /* 0-base on first 0 */
  static const UINT8 bits_ac_luminance[17] = {0, 0, 2, 1, 3, 3, 2, 4,   3,
                                              5, 5, 4, 4, 0, 0, 1, 0x7d};
  static const UINT8 val_ac_luminance[] = {
      0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06,
      0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
      0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0, 0x24, 0x33, 0x62, 0x72,
      0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
      0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45,
      0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
      0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75,
      0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
      0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3,
      0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
      0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9,
      0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
      0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1, 0xf2, 0xf3, 0xf4,
      0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa};

  /* 0-base on first 0 */
  static const UINT8 bits_ac_chrominance[17] = {0, 0, 2, 1, 2, 4, 4, 3,   4,
                                                7, 5, 4, 4, 0, 1, 2, 0x77};
  static const UINT8 val_ac_chrominance[] = {
      0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41,
      0x51, 0x07, 0x61, 0x71, 0x13, 0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
      0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0, 0x15, 0x62, 0x72, 0xd1,
      0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26,
      0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44,
      0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
      0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74,
      0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87,
      0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9a,
      0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4,
      0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
      0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda,
      0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf2, 0xf3, 0xf4,
      0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa};

  decompress_add_huff_table(cinfo, &cinfo->dc_huff_tbl_ptrs[0],
                            bits_dc_luminance, val_dc_luminance);
  decompress_add_huff_table(cinfo, &cinfo->ac_huff_tbl_ptrs[0],
                            bits_ac_luminance, val_ac_luminance);
  decompress_add_huff_table(cinfo, &cinfo->dc_huff_tbl_ptrs[1],
                            bits_dc_chrominance, val_dc_chrominance);
  decompress_add_huff_table(cinfo, &cinfo->ac_huff_tbl_ptrs[1],
                            bits_ac_chrominance, val_ac_chrominance);
}

void local_emit_message(jpeg_common_struct * /*cinfo*/, int /*msg_level*/) {
  return;
}

}  // namespace

ImageFormat GetFmt(DataRef data) {
  ImageFormat fmt;
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;

  cinfo.err = jpeg_std_error(&jerr);
  jerr.emit_message = local_emit_message;

  cinfo.out_color_space = JCS_RGB;
  jpeg_create_decompress(&cinfo);

  jpeg_mem_src(&cinfo, reinterpret_cast<unsigned char *>(
                           const_cast<char *>(data.data())),
               data.size());

  jpeg_read_header(&cinfo, TRUE);
  fmt.w = cinfo.image_width;
  fmt.h = cinfo.image_height;
  jpeg_destroy_decompress(&cinfo);
  return fmt;
}

// Returns true if successful false if an error was encountered.
bool ProcessJpeg(DataRef data, PixelRef *out) {
  /*
  TODO(parker): sort of error handling.
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;

  cinfo.err = jpeg_std_error( &jerr );
  jerr.emit_message   = local_emit_message;

  cinfo.out_color_space = JCS_RGB;
  jpeg_create_decompress( &cinfo );
  */

  static bool lost_camera_connect = false;
  struct jpeg_decompress_struct cinfo;

  // We set up the normal JPEG error routines, then override error_exit.
  JpegErrorManager jerr;
  cinfo.err = jpeg_std_error(&jerr.pub);
  jerr.pub.emit_message = local_emit_message;
  jerr.pub.error_exit = JpegErrorExit;
  // Establish the setjmp return context for my_error_exit to use.
  if (setjmp(jerr.setjmp_buffer)) {
    // If we get here, the JPEG code has signaled an error.
    if (!lost_camera_connect) {
      printf(
          "Lost camera connection in process_jpeg.\nLooking for reconnect "
          "...\n");
      fflush(stdout);
      lost_camera_connect = true;
    }
    jpeg_destroy_decompress(&cinfo);
    return false;
  }

  cinfo.out_color_space = JCS_RGB;
  jpeg_create_decompress(&cinfo);

  jpeg_mem_src(&cinfo, reinterpret_cast<unsigned char *>(
                           const_cast<char *>(data.data())),
               data.size());

  jpeg_read_header(&cinfo, TRUE);
  standard_huff_tables(&cinfo);

  /*printf( "JPEG File Information: \n" );
  printf( "Image width and height: %d pixels and %d pixels.\n",
  cinfo.image_width, cinfo.image_height );
  printf( "Color components per pixel: %d.\n", cinfo.num_components );
  printf( "Color space: %d.\n", cinfo.jpeg_color_space );
  printf("JpegDecompressed\n");*/

  jpeg_start_decompress(&cinfo);

  int offset = 0;
  int step = cinfo.num_components * cinfo.image_width;
  unsigned char *buffers[cinfo.image_height];
  for (size_t i = 0; i < cinfo.image_height; ++i) {
    buffers[i] = &reinterpret_cast<unsigned char *>(out)[offset];
    offset += step;
  }

  while (cinfo.output_scanline < cinfo.image_height) {
    jpeg_read_scanlines(&cinfo, &buffers[cinfo.output_scanline],
                        cinfo.image_height - cinfo.output_scanline);
  }

  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);

  if (lost_camera_connect) {
    printf("Camera connection restablished.\n");
    fflush(stdout);
    lost_camera_connect = false;
  }

  return true;
}

}  // namespace vision
}  // namespace aos
