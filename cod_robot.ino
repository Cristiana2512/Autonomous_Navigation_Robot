#include <AFMotor.h>
#include <Servo.h>

// Motoare
AF_DCMotor motor1(1); // Dreapta Spate
AF_DCMotor motor2(2); // Dreapta Fata
AF_DCMotor motor3(3); // Stanga Fata
AF_DCMotor motor4(4); // Stanga Spate

// Senzori IR 
const int senzorStanga  = A4;
const int senzorCentru  = 9;
const int senzorDreapta = A3;

//Ultrasonic
const int trigPin = A0;
const int echoPin = A2;

// servo
Servo cap;
const int servoPin = 10;

const int SERVO_STANGA  = 30;
const int SERVO_CENTRU  = 90;
const int SERVO_DREAPTA = 150;

const int VITEZA_DREPT = 160;
const int VITEZA_VIRAJ = 160;
const int VITEZA_CAUTE = 120;

const int DIST_OBSTACOL = 20;
const unsigned long TIMEOUT_LINIE = 1500;///TIMP MAXIM FARA LINIE

int ultimaDirectie = 0; // -1 st 0 centru 1 dr
unsigned long timpUltimaLinie = 0;//cand a detectat ultima oara linia

bool modOcolire = false;//daca ocoleste
int directieOcolire = 0; // -1 st, 1 dr



void setup() {
  Serial.begin(9600);

  pinMode(senzorStanga, INPUT);
  pinMode(senzorCentru, INPUT);
  pinMode(senzorDreapta, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  cap.attach(servoPin);//atasez la pin
  cap.write(SERVO_CENTRU);//pun servo in centru
  delay(400);

  setSpeedAll(VITEZA_DREPT);
  timpUltimaLinie = millis();

}

void loop() {
  int St = citireStabila(senzorStanga);
  int Ce = citireStabila(senzorCentru);
  int Dr = citireStabila(senzorDreapta);

  int dist = citesteDistanta();

  // Obstacol numai dacă linia e sub centru și nu suntem deja în ocolire
  if (Ce == 1 && dist < DIST_OBSTACOL && !modOcolire) {
    modOcolire = true;
    ocolesteObstacol();
    return;
  }

  if (modOcolire) return;//daca ocoleste nu face altceva

  // Urmărire linie
  if (St == 1 || Ce == 1 || Dr == 1) {
    timpUltimaLinie = millis();//actualizam daca senzorul citeste
  }

  if (Ce == 1) {
    inainte();
    ultimaDirectie = 0;
  } else if (St == 1) {
    vireazaStanga();
    ultimaDirectie = -1;
  } else if (Dr == 1) {
    vireazaDreapta();
    ultimaDirectie = 1;
  } else {
    if (millis() - timpUltimaLinie > TIMEOUT_LINIE) {
      stopRobot();
      return;
    }
    setSpeedAll(VITEZA_CAUTE);
    if (ultimaDirectie == -1) vireazaStanga();
    else if (ultimaDirectie == 1) vireazaDreapta();
    else inainte();
  }

  delay(5);//pauza pt procesor
}

//  Ocolire obstacol

void ocolesteObstacol() {
  stopRobot();
  delay(200);

  //scanare stanga
  cap.write(SERVO_STANGA);
  delay(350);
  int distSt = distMedie();

  //scanare dreapta
  cap.write(SERVO_DREAPTA);
  delay(350);
  int distDr = distMedie();

  // revine centru
  cap.write(SERVO_CENTRU);
  delay(250);

  // alege direcția ocolirii
  if (abs(distDr - distSt) < 5) {
    directieOcolire = ultimaDirectie != 0 ? ultimaDirectie : 1;
  } else {
    // LOGICA INVERS: dacă dreapta are distanța mai mare, ocolește STÂNGA (-1), altfel dreapta (1)
    directieOcolire = (distDr > distSt) ? -1 : 1;
  }

  Serial.print("Ocolire directie: ");
  Serial.println(directieOcolire == 1 ? "dreapta" : "stanga");

  // viraj primar + mers puțin în viraj (ca să nu stea pe loc)
  unsigned long tViraj1 = millis();
  while (millis() - tViraj1 < 300) {
    if (directieOcolire == 1) {
      vireazaDreapta();
    } else {
      vireazaStanga();
    }
    delay(20);
  }

  //  îndreptare + mers înainte puțin (ajustare direcție)
  unsigned long tIndreptare = millis();
  while (millis() - tIndreptare < 300) {
    if (directieOcolire == 1) {
      // Îndreptare viraj mic stânga
      setSpeedAll(VITEZA_VIRAJ);
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
      motor3.run(BACKWARD);
      motor4.run(BACKWARD);
      // Viraj mic stânga: roțile din stânga merg mai încet
      motor3.setSpeed(VITEZA_VIRAJ / 2);
      motor4.setSpeed(VITEZA_VIRAJ / 2);
      motor1.setSpeed(VITEZA_VIRAJ);
      motor2.setSpeed(VITEZA_VIRAJ);
    } else {
      // Îndreptare viraj mic dreapta
      setSpeedAll(VITEZA_VIRAJ);
      motor3.run(BACKWARD);
      motor4.run(BACKWARD);
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
      // Roțile din dreapta mai încet
      motor1.setSpeed(VITEZA_VIRAJ / 2);
      motor2.setSpeed(VITEZA_VIRAJ / 2);
      motor3.setSpeed(VITEZA_VIRAJ);
      motor4.setSpeed(VITEZA_VIRAJ);
    }
    delay(20);
  }

  // mers înainte puțin pentru a depăși obstacolul
  unsigned long tInainte = millis();
  while (millis() - tInainte < 400) {
    inainte();
    delay(20);
  }

  // viraj opus pentru a reveni spre linie, cu mers înainte în timpul virajului
  unsigned long tViraj2 = millis();
  while (millis() - tViraj2 < 700) {
    if (directieOcolire == 1) {
      vireazaStanga();
    } else {
      vireazaDreapta();
    }
    delay(20);
  }

  //  caută linia drept înainte
  unsigned long startCautare = millis();
  while (millis() - startCautare < 3000) {
    int St = citireStabila(senzorStanga);
    int Ce = citireStabila(senzorCentru);
    int Dr = citireStabila(senzorDreapta);

    if (St == 1 || Ce == 1 || Dr == 1) {
      timpUltimaLinie = millis();
      modOcolire = false;
      return;
    }

    inainte();
    delay(20);
  }

  // Daca nu gaseste linia, opreste robotul
  stopRobot();
  modOcolire = false;
  
}

//functii

int distMedie() {
  int suma = 0;
  for (int i = 0; i < 5; i++) {
    suma += citesteDistanta();
    delay(20);
  }
  return suma / 5;
}

int citireStabila(int pin) {
  int a = digitalRead(pin);
  delayMicroseconds(300);
  int b = digitalRead(pin);
  if (a == b) return a;
  else return 0;
}

int citesteDistanta() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long durata = pulseIn(echoPin, HIGH, 25000);
  if (durata == 0) return 100;

  return durata / 58;
}

void setSpeedAll(int v) {
  motor1.setSpeed(v);
  motor2.setSpeed(v);
  motor3.setSpeed(v);
  motor4.setSpeed(v);
}

void inainte() {
  setSpeedAll(VITEZA_DREPT);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void vireazaStanga() {
  setSpeedAll(VITEZA_VIRAJ);
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void vireazaDreapta() {
  setSpeedAll(VITEZA_VIRAJ);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

void stopRobot() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}
