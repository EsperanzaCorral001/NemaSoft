#ifndef LOGDISPLAY_H
#define LOGDISPLAY_H


#include <QWidget>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QPushButton>
#include <QHBoxLayout>
#include "logger.h" 

#include <QTextDocument>

class LogDisplay : public QWidget {
    Q_OBJECT

public:
    explicit LogDisplay(QWidget* parent = nullptr) : QWidget(parent) {
        setupUI();

        // Connect to the global logger
        connect(&Logger::instance(), &Logger::logMessage,
            this, &LogDisplay::appendLog, Qt::QueuedConnection);
    }

public slots:
    void appendLog(const QString& timestamp, const QString& level, const QString& message) {
        QString colorCode;

        if (level == "ERROR" || level == "CRITICAL") {
            colorCode = "#ff6b6b";  // Red
        }
        else if (level == "WARNING") {
            colorCode = "#ffd93d";  // Yellow
        }
        else if (level == "INFO") {
            colorCode = "#6bcf7f";  // Green
        }
        else if (level == "DEBUG") {
            colorCode = "#74b9ff";  // Blue
        }
        else {
            colorCode = "#d4d4d4";  // White
        }

        QString formattedMessage = QString(
            "<span style='color: #888;'>[%1]</span> "
            "<span style='color: %2;'>[%3]</span> "
            "<span style='color: #d4d4d4;'>%4</span>")
            .arg(timestamp)
            .arg(colorCode)
            .arg(level)
            .arg(message);

        m_logTextEdit->append(formattedMessage);

        // Auto-scroll to bottom
        QTextCursor cursor = m_logTextEdit->textCursor();
        cursor.movePosition(QTextCursor::End);
        m_logTextEdit->setTextCursor(cursor);
    }

    void clear() {
        m_logTextEdit->clear();
    }

private:
    void setupUI() {
        QVBoxLayout* mainLayout = new QVBoxLayout(this);

        // Text edit for logs
        m_logTextEdit = new QTextEdit(this);
        m_logTextEdit->document()->setMaximumBlockCount(1000);
        m_logTextEdit->setReadOnly(true);
        m_logTextEdit->setStyleSheet("background-color: #1e1e1e; color: #d4d4d4;");

        // Control buttons
        QHBoxLayout* buttonLayout = new QHBoxLayout();
        QPushButton* clearBtn = new QPushButton("Clear Logs", this);
        connect(clearBtn, &QPushButton::clicked, this, &LogDisplay::clear);
        buttonLayout->addWidget(clearBtn);

        mainLayout->addWidget(m_logTextEdit);
        mainLayout->addLayout(buttonLayout);
    }

    QTextEdit* m_logTextEdit;
};

#endif // LOGDISPLAY_H