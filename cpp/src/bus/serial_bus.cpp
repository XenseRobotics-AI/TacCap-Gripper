// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0

#include <taccap/bus/serial_bus.hpp>
#include <taccap/error.hpp>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <dirent.h>
#include <filesystem>
#include <utility>

namespace xense::taccap::bus {

namespace {

// Map a numeric baud rate to its B* constant. Returns 0 ("unsupported")
// for non-standard rates; the caller surfaces this as IoError. We only need
// the rates the firmware actually exposes (USART3 at 3 Mbps), so the
// standard <termios.h> set is sufficient — we avoid the asm/termbits.h
// BOTHER path that conflicts with libc's struct termios.
speed_t resolve_standard_baud(uint32_t b) noexcept {
    switch (b) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 230400:  return B230400;
        case 460800:  return B460800;
        case 500000:  return B500000;
        case 576000:  return B576000;
        case 921600:  return B921600;
        case 1000000: return B1000000;
        case 1152000: return B1152000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        case 2500000: return B2500000;
        case 3000000: return B3000000;
        case 3500000: return B3500000;
        case 4000000: return B4000000;
        default:      return 0;
    }
}

}  // namespace

SerialBus::SerialBus(const Config& cfg) : cfg_(cfg) {
    if (cfg_.device.empty()) {
        throw IoError("SerialBus: empty device path", EINVAL);
    }

    fd_ = ::open(cfg_.device.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (fd_ < 0) {
        throw IoError("SerialBus: open(" + cfg_.device + ")", errno);
    }

    // Make sure it actually is a TTY (not, e.g., a regular file).
    if (!::isatty(fd_)) {
        const int err = errno ? errno : ENOTTY;
        ::close(fd_);
        fd_ = -1;
        throw IoError(cfg_.device + " is not a tty", err);
    }

    // ONE OWNER PER PORT. Two handles on the same CDC port split its byte stream
    // between them: each reader gets half of every frame and both see CRC
    // errors, timeouts and "lost" ACKs that look like a link fault. The usual
    // culprit is a scan_grippers() run while a gripper is open -- in the same
    // GUI, or from a second terminal.
    //
    // flock() refuses a second SDK handle, in this process or another (the lock
    // belongs to the open file description, so a second open() here conflicts
    // too). TIOCEXCL then makes the kernel refuse any further open() of the
    // tty, which also covers tools that never heard of the lock (minicom,
    // screen, a stray pyserial). Root bypasses TIOCEXCL; flock still holds.
    if (::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
        const int err = errno;
        ::close(fd_);
        fd_ = -1;
        throw IoError("SerialBus: " + cfg_.device +
                      " is already open (another gripper handle or process owns it)",
                      err == EWOULDBLOCK ? EBUSY : err);
    }
    if (::ioctl(fd_, TIOCEXCL) != 0) {
        const int err = errno;
        ::close(fd_);
        fd_ = -1;
        throw IoError("SerialBus: TIOCEXCL on " + cfg_.device, err);
    }

    try {
        apply_termios_();
    } catch (...) {
        ::close(fd_);
        fd_ = -1;
        throw;
    }
}

SerialBus::~SerialBus() { close_(); }

SerialBus::SerialBus(SerialBus&& other) noexcept
    : cfg_(std::move(other.cfg_)), fd_(other.fd_) {
    other.fd_ = -1;
}

SerialBus& SerialBus::operator=(SerialBus&& other) noexcept {
    if (this != &other) {
        close_();
        cfg_      = std::move(other.cfg_);
        fd_       = other.fd_;
        other.fd_ = -1;
    }
    return *this;
}

void SerialBus::close_() {
    if (fd_ >= 0) {
        // TIOCEXCL outlives this fd: the kernel clears it only when the LAST
        // descriptor on the tty closes. If anything else holds the port
        // (ModemManager probing a fresh ttyACM, a monitor in another shell),
        // the flag would stay set and refuse our own next open with EBUSY.
        // Clear it explicitly; the flock goes with the fd.
        (void)::ioctl(fd_, TIOCNXCL);
        ::close(fd_);
        fd_ = -1;
    }
}

void SerialBus::apply_termios_() {
    struct termios tio{};
    if (::tcgetattr(fd_, &tio) != 0) {
        throw IoError("SerialBus: tcgetattr", errno);
    }

    // 8N1, no flow control, raw mode.
    tio.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);
    tio.c_cflag |= CS8 | CREAD | CLOCAL;

    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP |
                     INLCR  | IGNCR  | ICRNL | IXON | IXOFF | IXANY);
    tio.c_oflag &= ~(OPOST);
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    // Read timeout: VTIME is in 0.1s units, 0..255. We round up; <100 ms
    // requests are treated as "return as soon as anything is available"
    // (VMIN=0, VTIME=1 → 100 ms ceiling per read syscall).
    const unsigned ds = std::min(255u,
        std::max(1u, (cfg_.read_timeout_ms + 99) / 100));
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = static_cast<cc_t>(ds);

    const speed_t standard = resolve_standard_baud(cfg_.baudrate);
    if (standard == 0) {
        throw IoError("SerialBus: unsupported baud rate " +
                      std::to_string(cfg_.baudrate) +
                      " (no <termios.h> B* constant; need asm/termbits BOTHER path)",
                      EINVAL);
    }
    if (::cfsetospeed(&tio, standard) != 0 ||
        ::cfsetispeed(&tio, standard) != 0) {
        throw IoError("SerialBus: cfsetspeed", errno);
    }

    if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
        throw IoError("SerialBus: tcsetattr (baud=" +
                      std::to_string(cfg_.baudrate) + ")", errno);
    }

    // Drop anything left over from a previous user.
    ::tcflush(fd_, TCIOFLUSH);
}

std::size_t SerialBus::read(uint8_t* buf, std::size_t max) {
    if (fd_ < 0) throw IoError("SerialBus::read on closed device", EBADF);
    if (!buf || !max) return 0;

    const ssize_t n = ::read(fd_, buf, max);
    if (n >= 0) return static_cast<std::size_t>(n);

    if (errno == EAGAIN || errno == EINTR) return 0;
    throw IoError("SerialBus::read", errno);
}

std::vector<uint8_t> SerialBus::read(std::size_t max) {
    std::vector<uint8_t> out(max);
    const std::size_t n = read(out.data(), max);
    out.resize(n);
    return out;
}

void SerialBus::write(const uint8_t* data, std::size_t len) {
    if (fd_ < 0) throw IoError("SerialBus::write on closed device", EBADF);
    if (!data || !len) return;

    std::size_t total = 0;
    const auto deadline_ms = cfg_.write_timeout_ms;
    // Simple loop with a coarse deadline; for now we don't busy-wait.
    while (total < len) {
        struct pollfd pfd { fd_, POLLOUT, 0 };
        const int pr = ::poll(&pfd, 1, static_cast<int>(deadline_ms));
        if (pr == 0) {
            throw TimeoutError("SerialBus::write timeout", ETIMEDOUT);
        }
        if (pr < 0) {
            if (errno == EINTR) continue;
            throw IoError("SerialBus::write poll", errno);
        }
        const ssize_t n = ::write(fd_, data + total, len - total);
        if (n > 0) {
            total += static_cast<std::size_t>(n);
            continue;
        }
        if (n < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            throw IoError("SerialBus::write", errno);
        }
        // n == 0: nothing written and not blocked; treat as failure.
        throw IoError("SerialBus::write returned 0", EIO);
    }
}

void SerialBus::write(const std::vector<uint8_t>& bytes) {
    write(bytes.data(), bytes.size());
}

void SerialBus::flush_input() {
    if (fd_ >= 0) ::tcflush(fd_, TCIFLUSH);
}

void SerialBus::flush_output() {
    if (fd_ >= 0) ::tcflush(fd_, TCOFLUSH);
}

std::vector<std::string> SerialBus::list_ports() {
    namespace fs = std::filesystem;
    std::vector<std::string> out;

    // Prefer stable /dev/serial/by-id symlinks when present; they point to
    // /dev/ttyACM*/ttyUSB* but are stable across reboots.
    std::error_code ec;
    fs::path by_id = "/dev/serial/by-id";
    if (fs::is_directory(by_id, ec)) {
        for (auto& entry : fs::directory_iterator(by_id, ec)) {
            if (ec) break;
            out.push_back(entry.path().string());
        }
    }

    // Always fall back to scanning the raw nodes too, for boards where
    // udev doesn't populate /dev/serial/by-id.
    for (const char* prefix : {"/dev/ttyACM", "/dev/ttyUSB"}) {
        for (int i = 0; i < 32; ++i) {
            const std::string p = std::string(prefix) + std::to_string(i);
            struct stat st {};
            if (::stat(p.c_str(), &st) == 0 && S_ISCHR(st.st_mode)) {
                if (std::find(out.begin(), out.end(), p) == out.end()) {
                    out.push_back(p);
                }
            }
        }
    }
    return out;
}

}  // namespace xense::taccap::bus
