#include "TM4C123GH6PM.h"
/* 
	 LED Blinking ( LED Yakıp Söndürme ) 
	 Kerim Turak tarafindan yazilmistir ( Written by Kerim Turak )
	 Tüm haklari saklidir ( All rights reserved )
	 kerimturak@hotmail.com
	 10.07.2021
	 */	 
/* GPIO pinlerini kullanabilmemiz için öncelikle ilgili GPIO pinlerin 
	 baslatilmasi yani ayarlanmasi(yapilandirilmasi) gerekmektedir.*/
	 
/* Kullanilacak portlar için yapilandirma adimlari asagidaki gibidir

	1-Port veya çevrebirimin clock'un aktif edilmesi
	2-Port'un kilidinin açilmasi(unlock) eger kilitliyse. ***(Kilitli port pinleri için geçerli)
	3-Eger digital fonksiyon kullanilacaksa analog islevselligin deaktif edilmesi(disable) ***(Default olarak 0 yani deaktif) 
	4-Port control registerinin yapilandirilmasi (Portun GPIO olarak kullanilmasi için) ***(Default olarak 0 yani GPIO olarak ayarlanmistir portlar)
	5-Direction registerinin yapilandirilmasi(Portun pini input mu olacak output mu)
	6-DEN(digital enable) registerin yapilandirilmasi ( digital olarak kullanilacaksa eger)
	7-PUR(pull-up registerin ayarlanmasi) yada PDR(pull-down) eger gerekliyse tabi
	
	Yukaridaki adimlardan bazilarinin default degerleri dolayisiyla ayarlanmasina ihtiyaç yoktur.
	eger portun basit bir digital fonksiyon özelligi kullanilmak istenirse asasgidaki adimlarin takip edilmesi yeterlidir.
	*Clock ayarlanmalidir.
	*DIR registerinin ayarlanmalidir
	*EN(digital enable) registerinin ayarlanmalidir
	*/
/*				      Portun base adresi			RCGCGPIO registerinin offseti				RCGCGPIO'nun adres degeri
										Yani clock registiri
	Physical address of RCGCGPIO = 0x400FE000 			+ 		0x608        =                      0x400FE608
        RCGCGPIO'un fiziksel adresi 0x400FE000*/
# define SYSCTL_RCGCGPIO_R (*(( volatile unsigned long *)0x400FE608)) // pointer mantigi ile clock registestirinin adresine erisim

/*
	Base address of PORTF = 0x40025000
	Offset address of GPIODIR = 0x400 // // page number 663 TM4C123GH6PM datasheet
	GPIODIR Physical address =  0x40025000+0x400 = 0x40025400
*/
#define GPIO_PORTF_DIR_R  (*(( volatile unsigned long *)0x40025400 ) )
	
/*
	GPIO PORTF base address = 0x4002 5000
	GPIODEN Register offset address = 0x51c // page number 682 TM4C123GH6PM datasheet
	physical address = 0x40025000+0x51C = 0x4002551C
*/
# define GPIO_PORTF_DEN_R (*(( volatile unsigned long *)0x4002551C))

/* Son olarak hersey ayarlanmis durumda ama bir sorun var biz datayi nereden okuyacagiz yada nereye yazacagiz
	 Bunun için portun DATA registirini kullanacagiz
*/

/*
	PORTF Base Address  =   0x40025000
	GPIODATA Registe offset address = is 0x000
	GPIODATA  Physical address = 0x40025000+0x608 = 0x40025000
*/
#define GPIO_PORTF_DATA_R (*(( volatile unsigned long *)0x40025038))
int main ( void )
{
SYSCTL_RCGCGPIO_R |= 0x20; // Enable clock for PORTF
GPIO_PORTF_DEN_R  = 0x0E ;  // Enable PORTF Pin1, 2 and 3 as a digital pins
GPIO_PORTF_DIR_R  = 0x0E ;  // Configure ORTF Pin1, 2 and 3 digital output pins
		
	while (1) 
		{  
		GPIO_PORTF_DATA_R |= 0x02; // turn on red LED
		}
}

/*
PF1 – LED red
PF2 – LED blue
PF3 – LED green
DIR registirinda 1 degerini verdigimiz pinler output olurlar

GPIO_PORTF_DATA_R |= 0x04; // mavi LED
GPIO_PORTF_DATA_R |= 0x08; // yesil LED
*/

/*
	Ayrica asagidaki header file dosyasini kodumuza include ederek yani dahil ediyoruz register adreslerini 
	Bir mnemonic degiskenle adlandirma isinden kurtulabiliriz görüldügü gibi çok daha sade bir kod
*/

/*
#include "TM4C123.h" // header files contains memory addresses listing 
int main ( void )
{
SYSCTL->RCGCGPIO|= 0x20;
GPIOF->DIR = 0x02;
GPIOF->DEN = 0x02;	
		
	while (1){  
	GPIOF->DATA = 0x02; // turn on red LED
				
    }
}*/
