/*
DialogMotion - это класс диалогового окна, вызываемого при нажатии на кнопку "Режим съемки" основного окна.
Выполняет следующие функции:
1) Предоставляет возможность выбора определенного режима съемки при нажатии определенной кнопки
*/

#include "dialogmotion.h"
#include"QtHelp/QHelpContentItem"

DialogMotion::DialogMotion()
{
    setWindowTitle("Подготовка к съемке");
    QLabel *b3 = new QLabel("Режим съемки");
    radio1 = new QRadioButton(tr("Кадровая съемка"));
    radio2 = new QRadioButton(tr("Стереосъемка"));
    radio3 = new QRadioButton(tr("Площадная съемка"));
    radio4 = new QRadioButton(tr("Коридорная съемка"));

    B = new QLineEdit;
    L = new QLineEdit;

    ChangeButton = new QPushButton(tr("Активировать режим"));
    NavedDateTime = new QDateTimeEdit(QDateTime::currentDateTime().addSecs(1800));
    QVBoxLayout *vbl = new QVBoxLayout;

    vbl->addWidget(b3);
    vbl->addWidget(radio1); //
    vbl->addWidget(radio2); //
    vbl->addWidget(radio3); // Компоновка кнопок по вертикали
    vbl->addWidget(radio4); //
    QLabel *b2 = new QLabel("Время начала съемки");
    vbl->addWidget(b2);
    vbl->addWidget(NavedDateTime); //
    QLabel *b1 = new QLabel("Геодезические координаты точки начала съемки");
    QLabel *b = new QLabel("B (Геодезическая широта)");
    QLabel *l = new QLabel("L (Геодезическая долгота)");
    vbl->addWidget(b1);
    vbl->addWidget(b);
    vbl->addWidget(B);
    vbl->addWidget(l);
    vbl->addWidget(L);

    vbl->addWidget(ChangeButton);


    QHBoxLayout *hbl = new QHBoxLayout(this);
    QHBoxLayout *hbl1 = new QHBoxLayout;
    QHBoxLayout *hbl2 = new QHBoxLayout;
    QVBoxLayout *vbl1 = new QVBoxLayout;

    QLabel *phil = new QLabel("ФИ");
    phi_label  = new QLabel;
    phi_label->setMinimumSize(100,17);       // Задание размеров полей
    phi_label->setMaximumSize(100,17);

    vbl1->addWidget(phil);
    vbl1->addWidget(phi_label);

    QLabel *Rl = new QLabel("Расстояние между спутником и начальной точкой");
    R_label  = new QLabel;
    R_label->setMinimumSize(100,17);       // Задание размеров полей
    R_label->setMaximumSize(100,17);

    vbl1->addWidget(Rl);
    vbl1->addWidget(R_label);

    QLabel *wupr = new QLabel("Управляющее воздействие");
    omega_UPR_x_label = new QLabel;
    omega_UPR_y_label = new QLabel;                  // Создание полей
    omega_UPR_z_label = new QLabel;
    omega_UPR_x_label->setMinimumSize(100,17);
    omega_UPR_y_label->setMinimumSize(100,17);
    omega_UPR_z_label->setMinimumSize(100,17);       // Задание размеров полей
    omega_UPR_x_label->setMaximumSize(100,17);
    omega_UPR_y_label->setMaximumSize(100,17);
    omega_UPR_z_label->setMaximumSize(100,17);

    hbl1->addWidget(omega_UPR_x_label);
    hbl1->addWidget(omega_UPR_y_label);
    hbl1->addWidget(omega_UPR_z_label);

    vbl1->addWidget(wupr);
    vbl1->addLayout(hbl1);

    QLabel *LL = new QLabel("Поворот");
    Lambda0_label = new QLabel;
    Lambda0_label->setMinimumSize(100,17);       // Задание размеров полей
    Lambda0_label->setMaximumSize(100,17);
    Lambda1_label = new QLabel;
    Lambda1_label->setMinimumSize(100,17);       // Задание размеров полей
    Lambda1_label->setMaximumSize(100,17);
    Lambda2_label = new QLabel;
    Lambda2_label->setMinimumSize(100,17);       // Задание размеров полей
    Lambda2_label->setMaximumSize(100,17);
    Lambda3_label = new QLabel;
    Lambda3_label->setMinimumSize(100,17);       // Задание размеров полей
    Lambda3_label->setMaximumSize(100,17);

    hbl2->addWidget(Lambda0_label);
    hbl2->addWidget(Lambda1_label);
    hbl2->addWidget(Lambda2_label);
    hbl2->addWidget(Lambda3_label);

    vbl1->addWidget(LL);
    vbl1->addLayout(hbl2);

    hbl->addLayout(vbl);
    hbl->addLayout(vbl1);

}

void DialogMotion::update(double phi, double R, double w0, double w1, double w2,double l0,double l1,double l2,double l3){
    QString s0,s1,s2,s3,philine;

    philine  = QString::number(phi,'g',8);
    this->phi_label->setText(philine);

    philine  = QString::number(R,'g',8);
    this->R_label->setText(philine);

    // Запись значений r00 r01 и r02 в поля вывода координат КА
    s1    = QString::number(w0,'g',8);
    s2    = QString::number(w1,'g',8);
    s3    = QString::number(w2,'g',8);
    this->omega_UPR_x_label->setText(s1);
    this->omega_UPR_y_label->setText(s2);
    this->omega_UPR_z_label->setText(s3);

    // Запись значений w00 w01 и w02  TIME в поля вывода значенй угловой скорости в ск КА и в поле времени работы КА
    s0    = QString::number(l0,'g',8);
    s1    = QString::number(l1,'g',8);
    s2    = QString::number(l2,'g',8);
    s3    = QString::number(l3,'g',8);

    this->Lambda0_label->setText(s0);
    this->Lambda1_label->setText(s1);
    this->Lambda2_label->setText(s2);
    this->Lambda3_label->setText(s3);
}

