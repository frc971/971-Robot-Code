// for jpeglib.h
#include <stdio.h>

#include "aos/externals/libjpeg/include/jpeglib.h"

namespace frc971 {
namespace vision {

void decompress_add_huff_table (j_decompress_ptr cinfo,
    JHUFF_TBL **htblptr, const UINT8 *bits, const UINT8 *val);

void  standard_huff_tables (j_decompress_ptr cinfo);

void process_jpeg(unsigned char *out,unsigned char *image,size_t size);

}  // namespace vision
}  // namespace frc971
