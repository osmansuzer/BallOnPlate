



void setup(){

  Serial.begin(9600);
}

void loop(){

  if(Serial.available()){

      int n=0, incoming_size;
      char buf[1024]={0};

      while(n!=1)

        n+=Serial.readBytes(buf, 1);

      switch(*buf){

        case 0:
          incoming_size = 10;
          break;
        case 1:
          //PID
          break;

          /*
           * 
           */

        }
       
        while(n!=incoming_size)

         n+=Serial.readBytes(buf+n, 1);

       Serial.write(buf, n);
   }
  

  
}

