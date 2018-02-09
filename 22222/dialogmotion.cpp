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
    QHBoxLayout *hbl3 = new QHBoxLayout;
    QHBoxLayout *hbl4 = new QHBoxLayout;
    QHBoxLayout *hbl5 = new QHBoxLayout;
    QVBoxLayout *vbl1 = new QVBoxLayout;

    QLabel *kr = new QLabel("Крен");
    kren  = new QLabel;
    kren->setMinimumSize(100,17);
    kren->setMaximumSize(100,17);

    hbl4->addWidget(kr);
    hbl4->addWidget(kren);

    vbl1->addLayout(hbl4);

    QLabel *tg = new QLabel("Тангаж");
    tang  = new QLabel;
    tang->setMinimumSize(100,17);       // Задание размеров полей
    tang->setMaximumSize(100,17);

    hbl5->addWidget(tg);
    hbl5->addWidget(tang);

    vbl1->addLayout(hbl5);

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


    QLabel *gcp = new QLabel("Геодезические координаты текущей позиции визирования");
    geodPointB = new QLabel;
    geodPointL = new QLabel;                  // Создание полей
    geodPointH = new QLabel;
    geodPointB->setMinimumSize(100,17);
    geodPointL->setMinimumSize(100,17);
    geodPointH->setMinimumSize(100,17);       // Задание размеров полей
    geodPointB->setMaximumSize(100,17);
    geodPointL->setMaximumSize(100,17);
    geodPointH->setMaximumSize(100,17);

    hbl3->addWidget(geodPointB);
    hbl3->addWidget(geodPointL);
    hbl3->addWidget(geodPointH);

    vbl1->addWidget(gcp);
    vbl1->addLayout(hbl3);


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

void DialogMotion::updateGeodPoint(double b, double l, double H){
    QString ls,bs,hs;
    bs    = QString::number(b,'g',8);
    ls    = QString::number(l,'g',8);
    hs    = QString::number(H,'g',8);

    this->geodPointB->setText(bs);
    this->geodPointL->setText(ls);
    this->geodPointH->setText(hs);
}


