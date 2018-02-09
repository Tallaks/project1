 /*
Estar - это класс основного окна, в котором на данный момент можно выполнить следующие действия:
    1) Запуск моделирования движения спутника по круговой орбите с заданными параметрами.
    2) Вызов диалогового окна с выбором режима съемки для спутника
    3) Вызов окна, выводящего график зависимости угловой скорости по трем координатам от времени
    4) Вывод значений проекций угловой скорости по трем осям в системе координат КА, приборной системе координат, значения ИК датчика угловой скорости
    5) Вывод координат КА в ск J2000
    6) Вывод продолжительности работы спутника (текущего времени)
*/

#include "estar.h"
#include"QtWidgets"
#include <QThread>
#include <QFile>

//Count *c = new Count(this);

// Конструктор класса Estar основного окна
Estar::Estar(QWidget *parent)
    : QWidget(parent)
{

//Области группировки элементов по вертикали
    QVBoxLayout *vbl1 = new QVBoxLayout;            //  Объединение полей наклона и высоты
    QVBoxLayout *vbl2 = new QVBoxLayout;            //  Объединение полей даты и скорости
    QVBoxLayout *vbl3 = new QVBoxLayout;            //  Объединение полей координат КА и времени работы
    QVBoxLayout *vbl4 = new QVBoxLayout;            //  Объединение кнопки "Построить График" и элемента выбора выводимого графика
    QVBoxLayout *vbl5 = new QVBoxLayout;            //  Объединение объединенного поля информации и поля с кнопками
    QVBoxLayout *vbl6 = new QVBoxLayout;            //  Объединение всех элементов вывода угловой скорости
    QVBoxLayout *vbl7 = new QVBoxLayout;            //  Объединение кнопок "Старт" и "Режим Съемки"
    QVBoxLayout *vbl8 = new QVBoxLayout;            //  Объединение элементов начальной позиции КА

//Области группировки элементов по горизонтали
    QHBoxLayout *hbl1  = new QHBoxLayout;            //   "Высота в км" и "720"
    QHBoxLayout *hbl2  = new QHBoxLayout;            //   "Наклон в градусах" и "98"
    QHBoxLayout *hbl3  = new QHBoxLayout;            //   "Скорость в км/ч" и "7534.7744"
    QHBoxLayout *hbl4  = new QHBoxLayout;            //   Объединение разделов "Параметры орбиты", "Начальные параметры КА", объединенных полей координат и времени работы и раздела "Угловая скорость" по горизонтали
    QHBoxLayout *hbl5  = new QHBoxLayout;            //   Объединение всех элементов, содержащих кнопки
    QHBoxLayout *hbl6  = new QHBoxLayout;            //   Объединение полей вывода значений координат КА в J2000
    QHBoxLayout *hbl7  = new QHBoxLayout;            //   Объединение полей вывода значений проекций угловой скорости в с.к. КА
    QHBoxLayout *hbl8  = new QHBoxLayout;            //   Объединение полей вывода значений проекций угловой скорости в приборной с.к.
    QHBoxLayout *hbl9  = new QHBoxLayout;            //   Объединение полей вывода показаний ИК
    QHBoxLayout *hbl10 = new QHBoxLayout;            //   "Широта" и поле для ее ввода
    QHBoxLayout *hbl11 = new QHBoxLayout;            //   "Долгота" и поле для ее ввода
    QHBoxLayout *hbl12 = new QHBoxLayout;            //   "Текущий поворот по крену" и поле для ее ввода
    QHBoxLayout *hbl13 = new QHBoxLayout;            //   "Текущий поворот по тангажу" и поле для ее ввода
    QHBoxLayout *hbl14  = new QHBoxLayout;            //   Объединение полей вывода значений координат КА в J2000
    QHBoxLayout *hbl15  = new QHBoxLayout;            //   Расположение кнопок увеличения и уменьшения скорости обработки



// Создание подписи "Начальная высота в км"
    label1 = new QLabel;
    label1->setText("Высота в км");

// Создание записи "650"
    HeightLine = new QLabel;
    HeightLine->setText("650");

// Объединение двух записей по горизонтали
    hbl1->addWidget(label1);
    hbl1->addWidget(HeightLine);

// Создание подписи "Наклон в градусах"
    label2 = new QLabel;
    label2->setText("Наклон в градусах");

// Создание записи "98"
    InclineLine = new QLabel;
    InclineLine->setText("98");

// Объединение двух записей по горизонтали
    hbl2->addWidget(label2);
    hbl2->addWidget(InclineLine);

// Объединение полей наклона и высоты
    vbl1->addLayout(hbl1,0);
    vbl1->addLayout(hbl2,0);

// Внесение внутрь раздела окна "Параметры орбиты" объединенных полей наклона и высоты
    gb1 = new QGroupBox(this);                   // Создание раздела
    gb1->setLayout(vbl1);                        // Внесение данных внутрь раздела
    gb1->setTitle("Параметры орбиты");           // Задание имени раздела

// Создание подписи "Скорость в км/ч"
    label3 = new QLabel;
    label3->setText("Скорость в км/ч");

// Создание записи "7534.7744"
    VelocityLine = new QLabel;
    VelocityLine->setText("7534.7744");

// Объединение двух записей по горизонтали
    hbl3->addWidget(label3);
    hbl3->addWidget(VelocityLine);

// Создание подписи "Дата запуска"
    label4 = new QLabel;
    label4->setText("Дата запуска");

//  Создание элемента задания даты     (В текущей весрии программы не используется)
    StartDate = QDateTime::currentDateTime();
    dateEdit = new QDateTimeEdit(QDateTime::currentDateTime());                   // Создание элемента ввода вывода даты

    SpeedUpButton   = new QPushButton(">>");
    SpeedDownButton = new QPushButton("<<");
    label16   = new QLabel("Изменение скорости обработки");
    hbl15->addWidget(SpeedDownButton);
    hbl15->addWidget(SpeedUpButton);

// Создание подписи "Позиция КА в геодезических координатах"
    label10 = new QLabel;
    label10->setText("Позиция КА в геодезических координатах");

    vbl8->addWidget(label10);
    vbl8->addLayout(hbl10);
    vbl8->addLayout(hbl11);


//Объединение полей даты и скорости
    vbl2->addLayout(hbl3);
    vbl2->addLayout(vbl8);
    vbl2->addWidget(label4);
    vbl2->addWidget(dateEdit);
    vbl2->addWidget(label16);
    vbl2->addLayout(hbl15);


// Внесение внутрь раздела окна "Начальные параметры КА" объединенных полей даты и скорости
    gb2 = new QGroupBox(this);                  // Создание раздела
    gb2->setLayout(vbl2);                       // Внесение данных внутрь раздела
    gb2->setTitle("Начальные параметры КА");    // Задание имени раздела

// Создание подписи "Координаты КА в J2000"
   label5 = new QLabel;
   label5->setText("Координаты КА в J2000");

// Создание подписи "Время работы"
   label6 = new QLabel;
   label6->setText("Время работы");

// Создание полей вывода значений координат КА в J2000, задание минимального и максимального размера этих полей
   KoordinateLineX = new QLabel;
   KoordinateLineY = new QLabel;                // Создание полей
   KoordinateLineZ = new QLabel;
   KoordinateLineX->setMinimumSize(100,21);
   KoordinateLineY->setMinimumSize(100,21);
   KoordinateLineZ->setMinimumSize(100,21);     // Задание размеров полей
   KoordinateLineX->setMaximumSize(100,21);
   KoordinateLineY->setMaximumSize(100,21);
   KoordinateLineZ->setMaximumSize(100,21);

// Объединение полей вывода значений координат КА в J2000 по горизонтали
   hbl6->addWidget(KoordinateLineX);
   hbl6->addWidget(KoordinateLineY);
   hbl6->addWidget(KoordinateLineZ);

   label15 = new QLabel;
   label15->setText("Координаты КА в геод-ой ск");

// Создание полей вывода значений координат КА в J2000, задание минимального и максимального размера этих полей
   GeodLineL = new QLabel;
   GeodLineB = new QLabel;                // Создание полей
   GeodLineH = new QLabel;
   GeodLineL->setMinimumSize(100,21);
   GeodLineB->setMinimumSize(100,21);
   GeodLineH->setMinimumSize(100,21);     // Задание размеров полей
   GeodLineL->setMaximumSize(100,21);
   GeodLineB->setMaximumSize(100,21);
   GeodLineH->setMaximumSize(100,21);

// Объединение полей вывода значений координат КА в J2000 по горизонтали
   hbl14->addWidget(GeodLineL);
   hbl14->addWidget(GeodLineB);
   hbl14->addWidget(GeodLineH);

// Создание поля вывода времени работы КА, задание минимального размера поля
   TimeLine = new QLabel;                       // Создание поля
   TimeLine->setMinimumSize(100,21);            // Задание размеров поля

// Объединение полей координат и времени работы по вертикали
   vbl3->addWidget(label5);
   vbl3->addLayout(hbl6);
   vbl3->addWidget(label15);
   vbl3->addLayout(hbl14);
   vbl3->addLayout(hbl12);
   vbl3->addLayout(hbl13);
   vbl3->addWidget(label6);
   vbl3->addWidget(TimeLine);

// Создание подписи "В с.к. КА"
   label7 = new QLabel;
   label7->setText("В с.к. КА");

// Создание полей вывода значений проекций угловой скорости в с.к. КА, задание минимального и максимального размера этих полей
   omega_KA_x_label = new QLabel;
   omega_KA_y_label = new QLabel;                 // Создание полей
   omega_KA_z_label = new QLabel;
   omega_KA_x_label->setMinimumSize(100,17);
   omega_KA_y_label->setMinimumSize(100,17);
   omega_KA_z_label->setMinimumSize(100,17);
   omega_KA_x_label->setMaximumSize(100,17);      // Задание размеров полей
   omega_KA_y_label->setMaximumSize(100,17);
   omega_KA_z_label->setMaximumSize(100,17);

// Создание подписи "В с.к. прибора"
   label8 = new QLabel;
   label8->setText("В с.к. прибора");

// Создание полей вывода значений проекций угловой скорости в приборной с.к., задание минимального и максимального размера этих полей
   omega_PR_x_label = new QLabel;
   omega_PR_y_label = new QLabel;                  // Создание полей
   omega_PR_z_label = new QLabel;
   omega_PR_x_label->setMinimumSize(100,17);
   omega_PR_y_label->setMinimumSize(100,17);
   omega_PR_z_label->setMinimumSize(100,17);       // Задание размеров полей
   omega_PR_x_label->setMaximumSize(100,17);
   omega_PR_y_label->setMaximumSize(100,17);
   omega_PR_z_label->setMaximumSize(100,17);

// Объединение полей вывода значений проекций угловой скорости в с.к. КА по горизонтали
   hbl7->addWidget(omega_KA_x_label);
   hbl7->addWidget(omega_KA_y_label);
   hbl7->addWidget(omega_KA_z_label);

// Объединение полей вывода значений проекций угловой скорости в приборной с.к. по горизонтали
   hbl8->addWidget(omega_PR_x_label);
   hbl8->addWidget(omega_PR_y_label);
   hbl8->addWidget(omega_PR_z_label);

// Создание подписи "Показания ИК"
   label9 = new QLabel;
   label9->setText("Показания ИК");

// Создание полей вывода показаний ИК, задание минимального и максимального размера этих полей
   IK1_label = new QLabel;
   IK2_label = new QLabel;                          // Создание полей
   IK3_label = new QLabel;
   IK4_label = new QLabel;
   IK1_label->setMinimumSize(100,17);
   IK2_label->setMinimumSize(100,17);
   IK3_label->setMinimumSize(100,17);               // Задание размеров полей
   IK1_label->setMaximumSize(100,17);
   IK2_label->setMaximumSize(100,17);
   IK3_label->setMaximumSize(100,17);
   IK4_label->setMinimumSize(100,17);
   IK4_label->setMaximumSize(100,17);

// Объединение полей вывода показаний ИК по горизонтали
   hbl9->addWidget(IK1_label);
   hbl9->addWidget(IK2_label);
   hbl9->addWidget(IK3_label);
   hbl9->addWidget(IK4_label);

// Объединение всех элементов вывода угловой скорости по вертикали
   vbl6->addWidget(label7);
   vbl6->addLayout(hbl7);
   vbl6->addWidget(label8);
   vbl6->addLayout(hbl8);
   vbl6->addWidget(label9);
   vbl6->addLayout(hbl9);

// Внесение внутрь раздела окна "Угловая скорость" объединенных элементов вывода угловой скорости
   gb4 = new QGroupBox;                              // Создание раздела
   gb4->setLayout(vbl6);                             // Внесение данных внутрь раздела
   gb4->setTitle("Угловая скорость");                // Задание имени раздела

// Объединение разделов "Параметры орбиты", "Начальные параметры КА", объединенных полей координат и времени работы и раздела "Угловая скорость" по горизонтали
   hbl4->addWidget(gb1);
   hbl4->addWidget(gb2);
   hbl4->addLayout(vbl3);
   hbl4->addWidget(gb4);

// Создание кнопок
   StartButton =  new QPushButton("Старт");
   MotionButton = new QPushButton("Начать съемку...");
   StopButton =   new QPushButton("Стоп");
   QuitButton =   new QPushButton("Выход");
   PauseButton =  new QPushButton("Пауза");
   GraphButton =  new QPushButton("Построить график");

// Объединение кнопок "Старт" и "Режим Съемки" по вертикали
   vbl7->addWidget(StartButton);
   vbl7->addWidget(MotionButton);

// Создание элемента выбора выводимого графика
   GraphList << "В системе КА" << "В приборной системе" << "Измерительные каналы";                  // Создание списка вариантов
   GraphView = new QComboBox;                                                                       // Создание элемента выбора
   GraphView->addItems(GraphList);                                                                  // Добавление списка внутрь элемента выбора
   GraphView->setCurrentIndex(0);                                                                   // Задания выбора по умолчанию в позиции "В системе КА"

// Объединение кнопки "Построить График" и элемента выбора выводимого графика по вертикали
   vbl4->addWidget(GraphButton);
   vbl4->addWidget(GraphView);

// Внесение внутрь раздела окна "Графики угловых скоростей" объединенных кнопки "Построить график" и элемента выбора выводимого графика по вертикали
   gb3 = new QGroupBox;                             // Создание раздела
   gb3->setLayout(vbl4);                            // Внесение данных внутрь раздела
   gb3->setTitle("Графики угловых скоростей");      // Задание имени раздела

// Объединение всех элементов, содержащих кнопки, по горизонтали
   hbl5->addLayout(vbl7);
   hbl5->addWidget(PauseButton);
   hbl5->addWidget(StopButton);
   hbl5->addWidget(gb3);
   hbl5->addWidget(QuitButton);

// Объединение объединенного поля информации и поля с кнопками по вертикали
   vbl5->addLayout(hbl4);
   vbl5->addLayout(hbl5);

// Внести объединенные скомпонованные поля в окно программы
   setLayout(vbl5);

// Задание связей между сигналами нажатия кнопок и соответствующими слотами
   QObject::connect(QuitButton,&QPushButton::clicked,this,&Estar::QuitButton_clicked);
   QObject::connect(StartButton,&QPushButton::clicked,this,&Estar::StartButton_clicked);
   QObject::connect(PauseButton,&QPushButton::clicked,this,&Estar::PauseButton_clicked);
   QObject::connect(StopButton,&QPushButton::clicked,this,&Estar::StopButton_clicked);
   QObject::connect(GraphButton,&QPushButton::clicked,this,&Estar::GraphButton_clicked);
   QObject::connect(MotionButton,&QPushButton::clicked,this,&Estar::MotionButton_clicked);

// Создание элементов для графического окна, дополнительного потока, диалогового окна и расчетного класса
   DM = new DialogMotion;
 demo = new RealTimeZoomScroll;

   connect(DM->ChangeButton,&QPushButton::clicked,this,&Estar::movie_started);
// Задание стартового режима окна STOP
    previousMode = STOP;
    currentMode = STOP;

}
// Деструктор класса Estar основного окна

Estar::~Estar()
{

}

void Estar::Succesed(){
    S = new Succes;
    S->show();
}

void Estar::movie_started(){
    double lat,lon;
    lat = DM->B->text().toDouble();
    lon = DM->L->text().toDouble();
    QDateTime ndt = DM->NavedDateTime->dateTime();
    if(DM->radio1->isChecked()){
        emit send_kadr_position(lat,lon,1);
    }
    if(DM->radio2->isChecked()){
        emit send_kadr_position(lat,lon,2);
    }
    if(DM->radio3->isChecked()){
        emit send_kadr_position(lat,lon,3);
    }
    if(DM->radio4->isChecked()){
        emit send_kadr_position(lat,lon,4);
    }
}

// Описание слота нажатия на кнопку Старт
void Estar::StartButton_clicked(){
        changeMode(START);
}

// Описание слота нажатия на кнопку Пауза (Пока работает некорректно)
void Estar::PauseButton_clicked(){
    changeMode(PAUSE);
}

// Описание слота нажатия на кнопку Стоп (Пока работает некорректно)
void Estar::StopButton_clicked(){
    changeMode(STOP);
}

// Описание слота нажатия на кнопку График
void Estar::GraphButton_clicked(){
    demo->show();
}

// Описание слота нажатия на кнопку Выход
void Estar::QuitButton_clicked(){
   changeMode(STOP);
   close();
}

// Описание слота нажатия на кнопку Режим Съемки
void Estar::MotionButton_clicked(){
    DM->show();

}
// Функция смена вида основного окна при нажатии кнопок (Работает с ограничениями)
void Estar::changeMode(Mode mode){

    // Изменить текущий статус окна на mode
    currentMode = mode;

    switch (currentMode) {
        // Если нажата кнопка Старт
        case START:
            // Сделать кнопки Стоп и Пауза активными, кнопку Старт сделать неактивной

            PauseButton->setEnabled(true);
            StopButton->setEnabled(true);
            StartButton->setEnabled(false);
            break;

        // Если нажата кнопка Стоп
        case STOP:
            // Сделать кнопки Стоп и Пауза неактивными, кнопку Старт сделать активной
            PauseButton->setEnabled(false);
            StopButton->setEnabled(false);
            StartButton->setEnabled(true);
            previousMode = STOP;
            break;

        // Если нажата кнопка Пауза
        case PAUSE:
            // Сделать кнопки Стоп и Старт активными, кнопку Пауза сделать неактивной
            PauseButton->setEnabled(false);
            StopButton->setEnabled(true);
            StartButton->setEnabled(true);
            previousMode = PAUSE;
            break;
    }
}

// Функция, перезаписывающая значения координат КА, его угл. скорости и текущего времени в соответствующих строках основного окна
void Estar::update(double r00, double r01, double r02, double w00,double w01,double w02,QTime TIME){
    QString s0,s1,s2,s3,t;

    // Запись значений r00 r01 и r02 в поля вывода координат КА
    s1    = QString::number(r00,'g',8);
    s2    = QString::number(r01,'g',8);
    s3    = QString::number(r02,'g',8);
    this->KoordinateLineX->setText(s1);
    this->KoordinateLineY->setText(s2);
    this->KoordinateLineZ->setText(s3);

    // Запись значений w00 w01 и w02  TIME в поля вывода значенй угловой скорости в ск КА и в поле времени работы КА

    s0    = QString::number(w00,'g',8);
    s1    = QString::number(w01,'g',8);
    s2    = QString::number(w02,'g',8);

    this->omega_KA_x_label->setText(s0);
    this->omega_KA_y_label->setText(s1);
    this->omega_KA_z_label->setText(s2);
    t = TIME.toString("hh:mm:ss.zzz");
    this->TimeLine->setText(t);

}

void Estar::updategeod(double B, double L, double H){
    QString s1,s2,s3;
    // Запись значений r00 r01 и r02 в поля вывода координат КА
    s1    = QString::number(B,'g',8);
    s2    = QString::number(L,'g',8);
    s3    = QString::number(H,'g',8);
    this->GeodLineB->setText(s1);
    this->GeodLineL->setText(s2);
    this->GeodLineH->setText(s3);
}

void Estar::update1(double wnv0, double wnv1, double wnv2){
    QString s1,s2,s3;
    // Запись значений r00 r01 и r02 в поля вывода координат КА
    s1    = QString::number(wnv0,'g',8);
    s2    = QString::number(wnv1,'g',8);
    s3    = QString::number(wnv2,'g',8);
    this->omega_PR_x_label->setText(s1);
    this->omega_PR_y_label->setText(s2);
    this->omega_PR_z_label->setText(s3);
    //this->StartDate = ntd;
}

void Estar::update_ik(double ik1,double ik2,double ik3,double ik4){
    QString s1,s2,s3,s4;
    // Запись значений r00 r01 и r02 в поля вывода координат КА
    s1    = QString::number(ik1,'g',8);
    s2    = QString::number(ik2,'g',8);
    s3    = QString::number(ik3,'g',8);
    s4    = QString::number(ik4,'g',8);
    this->IK1_label->setText(s1);
    this->IK2_label->setText(s2);
    this->IK3_label->setText(s3);
    this->IK4_label->setText(s4);
}
