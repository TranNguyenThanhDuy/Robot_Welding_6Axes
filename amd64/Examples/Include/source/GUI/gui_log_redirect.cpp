#include "gui_log_redirect.h"

#include <QMetaObject>
#include <QPlainTextEdit>
#include <QString>

#include <ostream>

GuiLogStreambuf::GuiLogStreambuf(QPlainTextEdit* edit, std::streambuf* fallback)
    : edit_(edit), fallback_(fallback) {}

int GuiLogStreambuf::overflow(int ch) {
    if (ch == traits_type::eof()) {
        return traits_type::not_eof(ch);
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);
        buffer_.push_back(static_cast<char>(ch));
    }

    if (ch == '\n') {
        flushBuffer();
    }

    if (fallback_) {
        fallback_->sputc(static_cast<char>(ch));
    }

    return ch;
}

int GuiLogStreambuf::sync() {
    flushBuffer();
    if (fallback_) {
        fallback_->pubsync();
    }
    return 0;
}

void GuiLogStreambuf::flushBuffer() {
    std::string text;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        text.swap(buffer_);
    }

    if (text.empty()) {
        return;
    }
    if (!text.empty() && text.back() == '\n') {
        text.pop_back();
    }
    if (text.empty()) {
        return;
    }

    const QString line = QString::fromStdString(text);
    QMetaObject::invokeMethod(
        edit_,
        [edit = edit_, line]() { edit->appendPlainText(line); },
        Qt::QueuedConnection);
}

StreamRedirect::StreamRedirect(QPlainTextEdit* edit, std::ostream& stream)
    : stream_(stream), oldBuf_(stream.rdbuf()) {
    guiBuf_ = std::make_unique<GuiLogStreambuf>(edit, oldBuf_);
    stream_.rdbuf(guiBuf_.get());
}

StreamRedirect::~StreamRedirect() {
    stream_.rdbuf(oldBuf_);
}
