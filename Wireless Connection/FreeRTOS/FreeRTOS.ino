
boolean MyFlag = false;
TaskHandle_t Handle_Task1;
TaskHandle_t Handle_Task2;

void functionForTask1 (void* param);
void functionForTask2 (void* param);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  xTaskCreate(functionForTask1,"Task1",1000,NULL,1,&Handle_Task1);
  xTaskCreate(functionForTask2,"Task2",1000,NULL,1,&Handle_Task2);

  // Testing if it still works while setup runs
  Serial.print("Applying delay...");
  delay(2000);
  Serial.prinln("Done");
}

void loop() {
  // put your main code here, to run repeatedly:

}



void functionForTask1 (void* param){
  (void) param;

  TickType_t getTick;
  getTick = xTaskGetTickCount();

  while(true){
    Serial.print("Terminal 1\n");
    //vTaskDelay(1000/portTICK_PERIOD_MS);

    vTaskDelayUntil(&getTick,1000/portTICK_PERIOD_MS)
  }
}
void functionForTask2 (void* param){
  (void) param;
  int counter = 0;
  while(true){
    counter++;
    Serial.print("Terminal 2\n");
    vTaskDelay(1500/portTICK_PERIOD_MS);
    if (counter == 15){
      //vTaskDelete(Handle_Task1);
      vTaskSuspend(Handle_Task1);
    }
    if(counter == 10){
      vTaskResume(Handle_Task1);
    }
  }
}

