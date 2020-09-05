/*
 * util.c
 *
 *  Created on: 2020. 8. 17.
 *      Author: Kiwon
 */


#include	"util.h"

void	int2str(int n,char* buf,int len)
{
	int	i;

	for(i=0;i<len;i++)
	{
		buf[i] = '0';
	}

	for(i=len-1;i>=0&&n>0;n/=10,i--)
	{
		buf[i] = (char)(n%10+(int)'0');
	}
}
