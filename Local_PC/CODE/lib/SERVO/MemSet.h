#ifndef _MEMSET_H_
#define _MEMSET_H_
//===========================================================================
//	File Name	: MemSet.h
//	Description : Memory Copy/Reset Functions.
//===========================================================================

#ifndef memset

static void *memset_fcn(void *dest, int c, unsigned int count);
static void *memcpy_fcn(void *dest, const void *src, unsigned int count);

void *memset_fcn(void *dest, int c, unsigned int count)
{
	unsigned int i;

	if ((dest == 0) || (count <= 0))
		return 0;

	// for (i=0; i<count; i++)
	// 	(unsigned char)*((unsigned char*)dest + i) = c;

    unsigned char *byte_dest = (unsigned char *)dest; // Tạo con trỏ tạm thời

    for (i = 0; i < count; i++)
        byte_dest[i] = (unsigned char)c;

	return dest;
}

void *memcpy_fcn(void *dest, const void *src, unsigned int count)
{
	unsigned int i;

	// if ((dest == 0) || (count <= 0))
	// 	return 0;

	// for (i=0; i<count; i++)
	// 	(unsigned char)*((unsigned char*)dest + i) = (unsigned char)*((unsigned char*)src + i);

    if ((dest == 0) || (src == 0) || (count <= 0))
        return 0;

    unsigned char *byte_dest = (unsigned char *)dest; // Con trỏ tạm cho dest
    const unsigned char *byte_src = (const unsigned char *)src; // Con trỏ tạm cho src

    for (i = 0; i < count; i++)
        byte_dest[i] = byte_src[i]; // Sao chép dữ liệu từ src vào dest

	return dest;
}

#endif	// memset

#endif	// _MEMSET_H_