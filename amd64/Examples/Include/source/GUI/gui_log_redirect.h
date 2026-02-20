#pragma once

#include <iosfwd>
#include <memory>
#include <mutex>
#include <streambuf>
#include <string>

class QPlainTextEdit;

class GuiLogStreambuf : public std::streambuf {
public:
    GuiLogStreambuf(QPlainTextEdit* edit, std::streambuf* fallback);

protected:
    int overflow(int ch) override;
    int sync() override;

private:
    void flushBuffer();

    QPlainTextEdit* edit_;
    std::streambuf* fallback_;
    std::mutex mutex_;
    std::string buffer_;
};

class StreamRedirect {
public:
    StreamRedirect(QPlainTextEdit* edit, std::ostream& stream);
    ~StreamRedirect();

private:
    std::ostream& stream_;
    std::streambuf* oldBuf_;
    std::unique_ptr<GuiLogStreambuf> guiBuf_;
};

