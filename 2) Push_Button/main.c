#include "tm4c123gh6pm.h"
/*
	Tiva kartin üzerinde RGB LEDler ve 2 adet push button bulunmaktadir ve butonlar F portuna baglidirlar.
	*PF0 - sag buton
	*PF1 - kirmizi LED
	*PF2 - mavi LED
	*PF3 - yesil LED
	*PF4 - sol buton
	Kartin üzerindeki butonlar için harici bir pull up direnci olmadigi için biz dahili pull up veya pull down direnci yapilandirmaliyiz
	Bunun için PUR ve PDR registerlari kullanilir.
	Butonlar negatif mantikta çalisirlar yani eger butona basarsak low voltaj vermis oluruz, basmazsak high voltaj vermis oluruz.
	Ledler pozitif mantikla çalisir.
	PC[0:3], PD7, PE7, ve PF0 kilitli(LOCKED) registerlardir.
	*/
	
int main(void)
{
	//Tivanin header dosyasini dahil ettigimiz için artik sembolik isileri rahatlikla kullanabiliriz
	//Bu header file Valvanonun hazirladigi dosyadir.
	SYSCTL_RCGCGPIO_R |= 0x20;
	GPIO_PORTF_LOCK_R  = 0x4C4F434B; // ASCII degeri 'LOCK' 
	GPIO_PORTF_CR_R		 = 0x01; // Kilidi açildiktan sonra commit registerina 1 atiyarak DATA registrinin karsilik gelen bitinin artik yazilabilir oldugu bilgisini veriyoruz.
	GPIO_PORTF_PUR_R	|= 0x10; // Butonlar için dahili pull-down dirençleri aktif ettik.
	GPIO_PORTF_DIR_R  |= 0x02;
	GPIO_PORTF_DEN_R 	|= 0x12;
	GPIO_PORTF_LOCK_R  = 0x4C4F434B; // ASCII degeri 'LOCK' 
	GPIO_PORTF_CR_R		 = 0x11; // Kilidi açildiktan sonra commit registerina 1 atiyarak DATA registrinin karsilik gelen bitinin artik yazilabilir oldugu bilgisini veriyoruz.
	unsigned int state;
	while(1)
    {   
        state = GPIO_PORTF_DATA_R & 0x10;
        GPIO_PORTF_DATA_R = (~state>>3);    /* put it on red LED */
    }
}
