/***************************************************
This code can be added to your main file to add serial communication with python
 ****************************************************/
#define INPUT_SIZE 30 // define size of the input messages. Should be large enough
char input[INPUT_SIZE + 1];

void serialFlush(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}

const int inputs = 1;
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  delay(10);
}

void read_command(int* values){
    char * pch;
    byte size = Serial.readBytes(input, INPUT_SIZE);
    input[size] = 0;
    char* command = strtok(input, ",");
    // print command for debugging purposes
    Serial.print("command");Serial.println(command);
    //   Sent commands using 0:0    Which is equal to id:value
    while (command != 0)
    {

      // Split the command in two values
      char* separator = strchr(command, ':');
      if (separator != 0)
      {
          // Actually split the string in 2: replace ':' with 0
          *separator = 0;
          int index = atoi(command);
          // avoid sending multiple commands to same index
          if (values[index] != -1){
            continue;
          }
          ++separator;
          values[index] = atoi(separator);
      }
      // Find the next command in input string
      command = strtok(0, ",");
    }
    serialFlush();
}

void loop() {
  if(Serial.available()){
  // reading a byte from the input
//       byte size = Serial.readBytes(input, INPUT_SIZE);

      int values[inputs] = {0};
//    set the vals
      read_command(values);
      // do something with values array
    }
    // Wait a bit every loop. To not spam the servo or other actuator
    delay(15);
}