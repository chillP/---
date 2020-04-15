#include "softwareversion.h"
#include "loragw_user.h"

void SoftwareVersionShow(void)
{
  printf("Software Version:%02x-%02x-%02x %02d\r\n",SOFTWARE_VERSION_YEAR,SOFTWARE_VERSION_MONTH,SOFTWARE_VERSION_DAY,SOFTWARE_VERSION_NO);
	
	#ifdef F433
			printf("it's 433M software\r\n");
	#endif
	
	#ifdef F470
			printf("it's 470M software\r\n");
	#endif	
	
	#ifdef F868
			printf("it's 868M software\r\n");
	#endif	
	
	#ifdef F915
			printf("it's 915M software\r\n");
	#endif	
	
	#ifdef F470510
			printf("it's 470-510M software\r\n");
	#endif	
	
	#ifdef F433510
			printf("it's 433-510M software\r\n");
	#endif	
	printf("Min Freq:%06d Hz,Max Freq:%06d Hz\r\n",MIN_F,MAX_F);	
} 
