int  onewire_init(void);
void OWDepower(void);
void SetSpeed(int standard);
int OWTouchReset(void);
void OWWriteByte(int data, int power);
int OWReadByte(void);
int OWTouchByte(int data);
void OWBlock(unsigned char *data, int data_len);
int OWOverdriveSkip(unsigned char *data, int data_len);
