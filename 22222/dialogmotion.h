/*
DialogMotion - это класс диалогового окна, вызываемого при нажатии на кнопку "Режим съемки" основного окна.
Выполняет следующие функции:
1) Предоставляет возможность выбора определенного режима съемки при нажатии определенной кнопки
*/
#ifndef DIALOGMOTION_H
#define DIALOGMOTION_H

#include <QDialog>
#include <QPushButton>
#include <QRadioButton>
#include <QLineEdit>
#include <QLayout>
#include <QLabel>
#include <QTime>
#include <QDateTimeEdit>

class DialogMotion : public QDialog
{
    Q_OBJECT

public:
    DialogMotion();
    QRadioButton *radio0;
    QRadioButton *radio1;
    QRadioButton *radio2;
    QRadioButton *radio3;
    QRadioButton *radio4;
    QPushButton  *ChangeButton;
    QDateTimeEdit *NavedDateTime;

    QLabel *omega_UPR_x_label;
    QLabel *omega_UPR_y_label;
    QLabel *omega_UPR_z_label;
    QLabel *phi_label;
    QLabel *R_label;
    QLabel *Lambda0_label;
    QLabel *Lambda1_label;
    QLabel *Lambda2_label;
    QLabel *Lambda3_label;

    QLineEdit *B;
    QLineEdit *L;

public slots:
    void update(double phi, double R, double w0, double w1, double w2,double l0,double l1,double l2,double l3);

};

#endif // DIALOGMOTION_H
