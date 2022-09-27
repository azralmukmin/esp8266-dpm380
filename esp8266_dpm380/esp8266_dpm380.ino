#include <SoftwareSerial.h>

// modbus connection: (+) -> A, (-) -> B
#define RX  12    // D6 -> RXD RS485 module
#define TX  14    // D5 -> TXD

SoftwareSerial modbus;

unsigned char address[200];
int temp, time_modbus = 200;
unsigned long int data, pkWh, kVAh, p1, p2, p3, s1, s2, s3;
signed long int  q1, q2, q3, W, VA, VAR;
float pf, Hz, I1, I2, I3, In, v12, v23, v31, v1, v2, v3;
float thdI1, thdI2, thdI3, thdV1, thdV2, thdV3;

void setup() {
  Serial.begin(115200);

  modbus.begin(9600, SWSERIAL_8N1, RX, TX, false, 256); // serial baudrate, serial parity, rx, tx, invert, buffer capacity
}

void loop() {

  temp = 0;

  modbus_request(0x01, 0xA0, 0x58);  // Slave register, Start register, request register
  delay(time_modbus);
  modbus_receive();

  if (temp == 1) Serial.println("\n----Receive Success----\n");
  else Serial.println("\n----Receive Not Success----\n");

  delay(1000);
}

void modbus_request(unsigned char slave, unsigned char addr1, unsigned char addr2) {
  address[0] = slave; // slave id
  address[1] = 0x04;  // function read
  address[2] = 0x0F;  // start register address Hi
  address[3] = addr1; // start register address Low
  address[4] = 0x00;
  address[5] = addr2; // size register to read in hex
  unsigned int crc = CRC16(6);
  address[6] = crc & 0xFF;
  address[7] = (crc >> 8) & 0xFF;

  for (int i = 0; i < 8; i++) modbus.write(address[i]);

  int i = 0;
  while ((modbus.available() == 0) & (i < 100)) i++;
}

int CRC16(int DataLength) {
  unsigned int CheckSum = 0xFFFF;
  for (int m = 0; m < DataLength; m++) {
    CheckSum = CheckSum ^ address[m];
    for (int n = 0; n < 8; n++) {
      if ((CheckSum) & 0x0001 == 1) CheckSum = (CheckSum >> 1) ^ 0xA001;
      else CheckSum = CheckSum >> 1;
    }
  }
  return CheckSum;
}

void modbus_receive() {

  if (modbus.available()) {
    
    delay(time_modbus);
    int i = 0;
    while (modbus.available()) address[i++] = modbus.read();

    Serial.println(i);

    for (int j = 0; j < i; j++) {
      Serial.print(address[j], HEX);
      Serial.print(" ");
    }
    Serial.println();

    if (i == 181) {
      temp++;

      //(+) real energy
      data = 0;
      data = (data << 8) | address[7];
      data = (data << 8) | address[8];
      data = (data << 8) | address[9];
      data = (data << 8) | address[10];
      pkWh = data;
      Serial.print("\t pkWh = "); Serial.print(pkWh); Serial.println(" kWh");

      //Apparent energy
      data = 0;
      data = (data << 8) | address[15];
      data = (data << 8) | address[16];
      data = (data << 8) | address[17];
      data = (data << 8) | address[18];
      kVAh = data;
      Serial.print("\t kVAh = "); Serial.print(kVAh); Serial.println(" kVAh");

      //Total real power
      data = 0;
      data = (data << 8) | address[27];
      data = (data << 8) | address[28];
      data = (data << 8) | address[29];
      data = (data << 8) | address[30];
      W = data;
      Serial.print("\t W = "); Serial.print(W); Serial.println(" W");

      //Total apparent power
      data = 0;
      data = (data << 8) | address[31];
      data = (data << 8) | address[32];
      data = (data << 8) | address[33];
      data = (data << 8) | address[34];
      VA = data;
      Serial.print("\t VA = "); Serial.print(VA); Serial.println(" VA");

      //Total reactive power
      data = 0;
      data = (data << 8) | address[35];
      data = (data << 8) | address[36];
      data = (data << 8) | address[37];
      data = (data << 8) | address[38];
      VAR = data;
      Serial.print("\t VAR = "); Serial.print(VAR); Serial.println(" VAR");

      //Total power factor
      data = 0;
      data = (data << 8) | address[39];
      data = (data << 8) | address[40];
      pf = data * 0.001;
      Serial.print("\t pf = "); Serial.println(pf, 3);

      //Frequency
      data = 0;
      data = (data << 8) | address[41];
      data = (data << 8) | address[42];
      Hz = data * 0.01;
      Serial.print("\t Hz = "); Serial.print(Hz); Serial.println(" Hz");

      //Instantaneous current L1
      data = 0;
      data = (data << 8) | address[43];
      data = (data << 8) | address[44];
      data = (data << 8) | address[45];
      data = (data << 8) | address[46];
      I1 = data * 0.001;
      Serial.print("\t I1 = "); Serial.print(I1, 3); Serial.println(" A");

      //Instantaneous current L2
      data = 0;
      data = (data << 8) | address[47];
      data = (data << 8) | address[48];
      data = (data << 8) | address[49];
      data = (data << 8) | address[50];
      I2 = data * 0.001;
      Serial.print("\t I2 = "); Serial.print(I2, 3); Serial.println(" A");

      //Instantaneous current L3
      data = 0;
      data = (data << 8) | address[51];
      data = (data << 8) | address[52];
      data = (data << 8) | address[53];
      data = (data << 8) | address[54];
      I3 = data * 0.001;
      Serial.print("\t I3 = "); Serial.print(I3, 3); Serial.println(" A");

      //Instantaneous current Ln
      data = 0;
      data = (data << 8) | address[55];
      data = (data << 8) | address[56];
      data = (data << 8) | address[57];
      data = (data << 8) | address[58];
      In = data * 0.001;
      Serial.print("\t In = "); Serial.print(In, 3); Serial.println(" A");

      //Voltage phase L12
      data = 0;
      data = (data << 8) | address[59];
      data = (data << 8) | address[60];
      data = (data << 8) | address[61];
      data = (data << 8) | address[62];
      v12 = data * 0.1;
      Serial.print("\t v12 = "); Serial.print(v12, 1); Serial.println(" V");

      //Voltage phase L23
      data = 0;
      data = (data << 8) | address[63];
      data = (data << 8) | address[64];
      data = (data << 8) | address[65];
      data = (data << 8) | address[66];
      v23 = data * 0.1;
      Serial.print("\t v23 = "); Serial.print(v23, 1); Serial.println(" V");

      //Voltage phase L31
      data = 0;
      data = (data << 8) | address[67];
      data = (data << 8) | address[68];
      data = (data << 8) | address[69];
      data = (data << 8) | address[70];
      v31 = data * 0.1;
      Serial.print("\t v31 = "); Serial.print(v31, 1); Serial.println(" V");

      //Voltage phase L1
      data = 0;
      data = (data << 8) | address[71];
      data = (data << 8) | address[72];
      data = (data << 8) | address[73];
      data = (data << 8) | address[74];
      v1 = data * 0.1;
      Serial.print("\t v1 = "); Serial.print(v1, 1); Serial.println(" V");

      //Voltage phase L2
      data = 0;
      data = (data << 8) | address[75];
      data = (data << 8) | address[76];
      data = (data << 8) | address[77];
      data = (data << 8) | address[78];
      v2 = data * 0.1;
      Serial.print("\t v2 = "); Serial.print(v2, 1); Serial.println(" V");

      //Voltage phase L3
      data = 0;
      data = (data << 8) | address[79];
      data = (data << 8) | address[80];
      data = (data << 8) | address[81];
      data = (data << 8) | address[82];
      v3 = data * 0.1;
      Serial.print("\t v3 = "); Serial.print(v3, 1); Serial.println(" V");

      //Real power phase L1
      data = 0; 
      data = (data << 8) | address[83]; 
      data = (data << 8) | address[84]; 
      data = (data << 8) | address[85]; 
      data = (data << 8) | address[86]; 
      p1 = data;
      Serial.print("\t p1 = "); Serial.print(p1); Serial.println(" W");

      //Real power phase L2
      data = 0; 
      data = (data << 8) | address[87]; 
      data = (data << 8) | address[88]; 
      data = (data << 8) | address[89]; 
      data = (data << 8) | address[90]; 
      p2 = data;
      Serial.print("\t p2 = "); Serial.print(p2); Serial.println(" W");

      //Real power phase L3
      data = 0;
      data = (data << 8) | address[91]; 
      data = (data << 8) | address[92]; 
      data = (data << 8) | address[93]; 
      data = (data << 8) | address[94]; 
      p3 = data;
      Serial.print("\t p3 = "); Serial.print(p3); Serial.println(" W");

      //Apparent power phase L1
      data = 0;
      data = (data << 8) | address[95]; 
      data = (data << 8) | address[96]; 
      data = (data << 8) | address[97]; 
      data = (data << 8) | address[98]; 
      s1 = data;
      Serial.print("\t s1 = "); Serial.print(s1); Serial.println(" VA");

      //Apparent power phase L2
      data = 0;
      data = (data << 8) | address[99]; 
      data = (data << 8) | address[100]; 
      data = (data << 8) | address[101]; 
      data = (data << 8) | address[102]; 
      s2 = data;
      Serial.print("\t s2 = "); Serial.print(s2); Serial.println(" VA");

      //Apparent power phase L3
      data = 0;
      data = (data << 8) | address[103]; 
      data = (data << 8) | address[104]; 
      data = (data << 8) | address[105]; 
      data = (data << 8) | address[106]; 
      s3 = data;
      Serial.print("\t s3 = "); Serial.print(s3); Serial.println(" VA");

      //Reactive power phase L1
      data = 0;
      data = (data << 8) | address[107]; 
      data = (data << 8) | address[108]; 
      data = (data << 8) | address[109]; 
      data = (data << 8) | address[110]; 
      q1 = data;
      Serial.print("\t q1 = "); Serial.print(q1); Serial.println(" VAR");

      //Reactive power phase L2
      data = 0;
      data = (data << 8) | address[111]; 
      data = (data << 8) | address[112]; 
      data = (data << 8) | address[113]; 
      data = (data << 8) | address[114]; 
      q2 = data;
      Serial.print("\t q2 = "); Serial.print(q2); Serial.println(" VAR");

      //Reactive power phase L3
      data = 0;
      data = (data << 8) | address[115]; 
      data = (data << 8) | address[116]; 
      data = (data << 8) | address[117]; 
      data = (data << 8) | address[118]; 
      q3 = data;
      Serial.print("\t q3 = "); Serial.print(q3); Serial.println(" VAR");

      //THD current L1
      data = 0;
      data = (data << 8) | address[167];
      data = (data << 8) | address[168];
      thdI1 = data * 0.1;
      Serial.print("\t thdI1 = "); Serial.print(thdI1, 1); Serial.println(" %");

      //THD current L2
      data = 0;
      data = (data << 8) | address[169];
      data = (data << 8) | address[170];
      thdI2 = data * 0.1;
      Serial.print("\t thdI2 = "); Serial.print(thdI2, 1); Serial.println(" %");

      //THD current L3
      data = 0;
      data = (data << 8) | address[171];
      data = (data << 8) | address[172];
      thdI3 = data * 0.1;
      Serial.print("\t thdI3 = "); Serial.print(thdI3, 1); Serial.println(" %");

      //THD voltage L1
      data = 0;
      data = (data << 8) | address[173];
      data = (data << 8) | address[174];
      thdV1 = data * 0.1;
      Serial.print("\t thdV1 = "); Serial.print(thdV1, 1); Serial.println(" %");

      //THD voltage L2
      data = 0;
      data = (data << 8) | address[175];
      data = (data << 8) | address[176];
      thdV2 = data * 0.1;
      Serial.print("\t thdV2 = "); Serial.print(thdV2, 1); Serial.println(" %");

      //THD voltage L3
      data = 0;
      data = (data << 8) | address[177];
      data = (data << 8) | address[178];
      thdV3 = data * 0.1;
      Serial.print("\t thdV3 = "); Serial.print(thdV3, 1); Serial.println(" %");
    }
  }
}
