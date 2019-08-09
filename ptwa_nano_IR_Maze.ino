/**
   慧燈2019PTWA軌道障礙賽基本程式
   使用4顆 Sharp IR GP2Y0A41SK0F 紅外線測距模組

*/

/*********************************************************************************
   感應器初始化區 sensor measurement
********************************************************************************/
#define RIGHTEYE A0
#define MIDLEFTEYE A1
#define MIDRIGHTEYE A4
#define LEFTEYE A3
const int RIGHT = 0, MID = 1, MIDLEFT = 1, MIDRIGHT = 2, LEFT = 3;
int sensorValue[4] = {0}; //RIGHT = 0, MID = 1, MIDLEFT = 1, MIDRIGHT = 2, LEFT = 3

/********************************************************************************
   車子規格區 car measurement
*******************************************************************************/
const int wd = 1070;//兩輪中心距離 in 1/100cm
const int carWidth = 1310; //車身寬度 in 1/100cm
const int carLen = 1180; //車身長度 in 1/100cm

/********************************************************************************
   速度、距離、PID 區
*******************************************************************************/




//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//tP速度下的車子參數 分區
int tP = 170;//基本速度
const int turn90MS = 100;//轉90度所需時間
const int safeDistance = 1000; //車速=tP時，看到障礙物到完全煞停的所需距離 用safeDistTest()測,1/100 cm
const int turnSD = safeDistance + 300; //障礙物前煞停+轉90度， 用turnSDTest()測,1/100 cm
const int carVec = 5000;//車子以 tP 前進1秒的距離，1/100cm per second
//PID
const int pivot = 1700;//前進時要走在離牆多遠的線上 單位:1/100 cm, mm*10 cos(30)=0.866 cos(60)=0.5
float kP = 0.4;//正比於err造成的輪速改變 => 影響誤差反應幅度。通常是小數
float kD = 0.6;//誤差的微分
const int errMax = int(tP / kP *0.89); //0.89); //err上限，影響 tP 可變範圍，越大則 tP 可變範圍越大，則可做出越尖銳的轉彎
const int errDeadzone = 10; // 1/100 cm, 反應死區

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^



/*
  //vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
  //tP速度下的車子參數 分區
  int tP = 70;
  const int turn90MS = 410;//轉90度所需時間
  const int pivot = 1700;//前進時要走在離牆多遠的線上 單位:1/100 cm, mm*10 cos(30)=0.866 cos(60)=0.5
  float kP = 0.2;//正比於err造成的轉彎幅度
  float kD = 0;
  const int errMax = int(tP / kP / 1.2); //越小則轉彎半徑越大
  const int errDeadzone = 10; // 1/100 cm
  const int safeDistance = 500; //tp=時，看到障礙物到完全煞停的所需距離 用safeDistTest()測,1/100 cm
  const int turnSD = safeDistance + 300; //障礙物前煞停+轉90度， 用turnSDTest()測,1/100 cm
  const int carVec = 3000;//車子以 tP 前進1秒的距離，1/100cm per second
  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
*/

//PD初始化
int err = 0;//誤差
int lastErr = 0;//上迴圈的誤差
int deltaErr = 0;//本次與上次迴圈誤差的差值

/*********************************************************************************
  設定靠左/右牆區
  後續在程式中可以隨時切換，但是要注意
  1.不可以離要靠的那面牆太遠(<=pivot*1.5)
  2.紅外線如果朝超過3點鐘方向會造成迴轉
********************************************************************************/
//int FOLLOWWALL = LEFT;
int FOLLOWWALL = RIGHT;//設定靠左/右牆


/*********************************************************************************
  主程式從這裡開始會執行setup()一次，然後進loop()無限循環
 *********************************************************************************/

/**
 * 起始設定，讀第12隻腳決定開機後要沿哪一牆，HIGH=LEFT, LOW=RIGHT;
 */
void setup()
{
  Serial.begin(115200);
  pinMode(LEFTEYE, INPUT);
  pinMode(MIDLEFTEYE, INPUT);
  pinMode(RIGHTEYE, INPUT);
  pinMode(MIDRIGHTEYE, INPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  (digitalRead(12) == 1) ? FOLLOWWALL = LEFT : FOLLOWWALL = RIGHT;
}

/**
 * 無限迴圈的主程式
 */
void loop()
{
 
  
  //motors(100, 100);//Leo on 20190729

  /*每次tP改變或是換車體時，請做以下測試，並將所得數值填入上方 【tP速度下的車子參數 分區】
     測試OKed
     turn90Test(280);
     safeDistTest(1000);
     turnSDTest(safeDistance + 300);
     carVecTest();//=500
     readSensors();//這兩行拿一個紙箱測試PID直到能沿邊繞
     moveForward();//這兩行拿一個紙箱測試PID直到能沿邊繞
  */


  
    readSensors();
    if ( isFrontWalled() && isRightWalled() ) {
      pingandLeftTurn90();
    }
    else if(!isRightWalled()){
      motorsMs(tP,0,30);//右邊沒牆，等輪子走到轉角
      while(!isRightWalled()) moveForward();
    }
    else {
      moveForward();
    }
 



  /**
    TODO 尚未完成
  */
  /*
    //靠牆走
    //如果遇到障礙物
    //切換到另一邊牆
    if (isPathForward()) {
    moveForward();
    }
    else {
    if (FOLLOWWALL == RIGHT) {
    brake();
    //turnLeft();
    shiftLaneTo(LEFT);
    FOLLOWWALL = LEFT;
    }
    else {
    brake();
    //turnRight();
    shiftLaneTo(RIGHT);
    FOLLOWWALL = RIGHT;
    }
    }
  */
}


//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//以下是基本功能區，包含有 1.車子基本動作 2.馬達函數區 3.感測器區 4.車子檢測用函數區
//                       5.PID用函數區

/********************************************************************************
  1.車子基本動作區 movement functions
*******************************************************************************/

/**
 * 扶著牆往前走，使用前需呼叫 readSensors()
 */
void moveForward() //
{
  if (FOLLOWWALL == LEFT) {//左手法則
    err = pivot -  sensorValue[LEFT] ;
    err = crampErr(err);
    deltaErr = err -  lastErr;
    lastErr = err;
    motors(int(tP + err * kP + deltaErr * kD ), int(tP - err * kP - deltaErr * kD));
  }
  else if (FOLLOWWALL == RIGHT) { //右手法則
    err = pivot -  sensorValue[RIGHT];
    err = crampErr(err);
    deltaErr = err -  lastErr;
    lastErr = err;
    motors(int(tP - err * kP - deltaErr * kD), int(tP + err * kP + deltaErr * kD));
  }
}

/**
 * TODO 尚未完成 換跑道
 */
void shiftLaneTo(int LA)
{
  int a; (LA == LEFT) ? a = 1 : a = -1;
  int v = tP;
  motorsMs(-v * a, v * a, turn90MS / 2); //轉45度 cos(45) = 0.525
  motorsMs(v, v, pm2TimeMS(150));//前進 in 1/100cm
  motorsMs(v * a, -v * a, turn90MS / 1.25); //抵消慣性轉45度
  //brake();
}

/**
 * 往dir方向盲轉90度，dir=RIGHT或LEFT
 */
void turn90(int dir)
{
  int a;
  (dir == LEFT) ? a = 1 : a = -1;
  motorsMs(-a * tP, a * tP, turn90MS);
}

/**
 * 左轉90度，時間用盡 或 前面沒牆了 則停止
 */
void pingandLeftTurn90()
{
  int tt = turn90MS;
  //brake();
  readSensors();
  motors(-tP, tP);
  while (tt > 0 && isFrontWalled()) {
    tt -= 1;
    delay(1);
  }
}

/**
 * s in 1/100 cm 輸入距離s轉為以 carVec 前進所需ms
 * 輸出:ms
 */
int pm2TimeMS(int s) //
{
  float error = 200.0;//實測誤差約少20mm
  return int((s + error) / carVec * 1000 ) ;
}

/********************************************************************************
   2.馬達函數區 motor functions
*******************************************************************************/

/**
   motorsMs(左馬達出力, 右馬達出力, 持續毫秒數)
   函數執行完畢，馬達狀態會鎖住，除非呼叫其他馬達函數。
*/
void motorsMs(int L, int R, int ms) //驅動馬達(左馬達出力, 右馬達出力,持續ms)
{
  motors(L, R);
  delay(ms);
}

/**
   motors(左馬達出力, 右馬達出力)
   函數執行完畢，馬達狀態會鎖住，除非呼叫其他馬達函數。
*/
void motors(int L, int R) //驅動馬達(左馬達出力, 右馬達出力)
{
  /*
    Serial.print(L);
    Serial.print(" --- ");
    Serial.println(R);
  */
  int bias = 0;
  digitalWrite(4, L >= 0 ? 0 : 1);
  analogWrite(5, L >= 0 ? min(L, 255) : min(255 + L, 255));
  digitalWrite(2, R >= 0 ? 0 : 1);
  analogWrite(3, R >= 0 ? min(R - bias, 255) : min(255 + R - bias, 255));
}

/**
   馬達不出力，滑行n毫秒
*/
void coast(int n)//馬達滑行
{
  digitalWrite(2, 0);
  analogWrite(3, 0);
  digitalWrite(4, 0);
  analogWrite(5, 0);
  delay(n);
}

/*
   馬達煞停，會有立即的停頓感
*/
void brake() //馬達煞停
{
  digitalWrite(2, 1);
  analogWrite(3, 255);
  digitalWrite(4, 1);
  analogWrite(5, 255);
}

/*
  煞車後維持停止
 */
void stop()
{
  while(true) brake();
}


/*
  ABSStop(煞車持續時間)，輸入10的倍數
  持續點踩煞車一段時間，執行完不一定車子停止。
*/
void ABSStop(unsigned int ms) {
  int tt = 0;
  while (tt < ms) {
    brake();
    delayMicroseconds(9000);
    motors(0, 0);
    delayMicroseconds(1000);
    tt += 10;
  }
}
/*******************************************************************************
   3.感測器區 sensor functions
********************************************************************************/

/**
 * 讀所有感應器，讀完會儲存在sensorValue[RIGHT],sensorValue[MIDRIGHT],
 * sensorValue[MIDLEFT],sensorValue[LEFT]四個格子中
 */
void readSensors()
{
  sensorValue[RIGHT] = getDistanceCentCM((analogRead(RIGHTEYE) + analogRead(RIGHTEYE)) / 2);
  sensorValue[MIDRIGHT] = getDistanceCentCM((analogRead(MIDRIGHTEYE) + analogRead(MIDRIGHTEYE)) / 2);
  sensorValue[MIDLEFT] = getDistanceCentCM((analogRead(MIDLEFTEYE) + analogRead(MIDLEFTEYE)) / 2);
  sensorValue[LEFT] = getDistanceCentCM((analogRead(LEFTEYE) + analogRead(LEFTEYE)) / 2);
  Serial.print(sensorValue[LEFT]); Serial.print(",");
  Serial.print(sensorValue[MIDLEFT]); Serial.print(",");
  //Serial.print(sensorValue[MIDRIGHT]); Serial.print(",");
  Serial.println(sensorValue[RIGHT]);

}

/**
 * 讀一次右眼，判斷右邊有牆嗎?判斷條件:sensorValue[RIGHT] < pivot * 1.5
 * 輸出: 有牆=true, 沒牆=false
 */
bool isRightWalled()  //
{
  sensorValue[RIGHT] = getDistanceCentCM(analogRead(RIGHTEYE));
  if (sensorValue[RIGHT] < pivot * 1.2) return true;
  else return false;
}

/**
 * 讀一次前眼，判斷前面有牆嗎?判斷條件:sensorValue[MIDLEFT] <= turnSD
 * 輸出: 有牆=true, 沒牆=false
 */
bool isFrontWalled()  //
{
  sensorValue[MIDLEFT] = getDistanceCentCM(analogRead(MIDLEFTEYE));
  if (sensorValue[MIDLEFT] <= turnSD) return true;
  else return false;
}

/**
 * 紅外線伏特換算成距離，單位為1/100公分
 * reference: https://github.com/jeroendoggen/Arduino-distance-sensor-library
 */
int getDistanceCentCM(int v)
{
  int adcValue = v;
  if (adcValue > 600)                             // lower boundary: 4 cm (3 cm means under the boundary)
  {
    return (3 * 100);
  }

  if (adcValue < 80 )                             //upper boundary: 36 cm (returning 37 means over the boundary)
  {
    return (37 * 100);
  }

  else
  {
    //return (1 / (0.000413153 * adcValue - 0.0055266887))*100;
    return (1 / (0.000413153 * adcValue - 0.0055266887)) * 100;
  }
}


/*****************************************************************
   4.車子檢測用函數區
 *****************************************************************/

/**
 * 測試PWM功能是否正常
 */
void pwmTest()
{
  for (int i = 0; i <= 200; i += 10) {
    motorsMs(i, i, 1000);
    brake();
  }
}

/**
 * 車子在dd距離時開始煞車，用來取得safeDistance
 */
void safeDistTest(int dd)
{
  readSensors();
  if (sensorValue[MID] <= dd ) stop();
  else motors(tP, tP);
}

/*
車子在ss距離時開始煞停+轉90度，用來取得turnSD
 */
void turnSDTest(int ss)
{
  readSensors();
  if (sensorValue[MID] <= ss)
  {
    motorsMs(-tP, tP, turn90MS); //左轉90
    stop();
  }
  else {
    motors(tP, tP);
  }
}

/*
車子持續迴轉 ms 後停止，用以取得迴轉90度時間
 */
void turn90Test(int ms)
{
  motorsMs(-tP, tP, ms);
  stop();
}

/*
測車子每秒能走多遠
 */
void carVecTest()
{
  motorsMs(tP, tP, 1000);
  stop();
}


/*****************************************************************
   5.PID用函數區
 *****************************************************************/
int crampErr(int err) //限制err的上下限   reference 龍華科技大學第五代迷宮電腦鼠論文
{
  if (err > errDeadzone) {
    err -= errDeadzone;
    err = min(errMax, err);
  }
  else if (err < -errDeadzone) {
    err += errDeadzone;
    err = max(-errMax, err);
  }
  else err = 0;
  return err;
}
