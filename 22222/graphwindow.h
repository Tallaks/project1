#ifndef GRAPHWINDOW_H
#define GRAPHWINDOW_H

#include <QWidget>
#include <QTextEdit>
#include <QtOpenGL/QtOpenGL>
#include <QScrollBar>
#include "qcustomplot.h"

class GraphWindow : public QWidget
{
    Q_OBJECT

public:
    explicit GraphWindow(QWidget *parent = 0);

    void setupRealtimeDataDemo(QCustomPlot *customPlot);
    QCustomPlot *customPlot;

private:
    QScrollBar *VerticalScrollBar;
    QScrollBar *HorisontalScrollBar;
    QTimer dataTimer;
    QCPItemTracer *itemDemoPhaseTracer;

private slots:
    void realtimeDataSlot(double y1, double y2, double y3, double x);
    void horzScrollBarChanged(int value);
    void vertScrollBarChanged(int value);
    void xAxisChanged(QCPRange range);
    void yAxisChanged(QCPRange range);

};

#endif // GRAPHWINDOW_H
