/*
 * tileswriter.hpp
 *
 * Atlas at LAAS
 *
 * author:  Pierrick Koch <pierrick.koch@laas.fr>
 * created: 2015-03-09
 * license: BSD
 */
#ifndef ATLAAS_TILES_WRITER_HPP
#define ATLAAS_TILES_WRITER_HPP

#include <queue>
#include <utility>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <functional>

namespace atlaas {

template <typename T> class wqueue {
    std::queue<T> qu;
    std::mutex mt;
    std::condition_variable cv;

public:
    void push(T& item) {
        std::lock_guard<std::mutex> lk(mt);
        qu.push(item);
        cv.notify_one();
    }
    T pop() {
        std::unique_lock<std::mutex> lk(mt);
        cv.wait(lk, [=]{return !qu.empty();});
        T item = qu.front();
        qu.pop();
        return item;
    }
    size_t size() {
        std::lock_guard<std::mutex> lk(mt);
        size_t size = qu.size();
        return size;
    }
};

class tiles_io {
    wqueue<std::array<int, 2>> tiles;
    std::function<void(int, int)> io;
    std::vector<std::thread> tharr;
    bool running = true;

public:
    void launch(size_t n) {
        tharr.resize(n);
        for (size_t i = 0; i < n; i++) {
            tharr[i] = std::thread(&tiles_io::run, this);
        }
    }
    void todo(const std::function<void(int, int)>& _io) {
        io = _io;
    }
    void process(int x, int y) {
        std::array<int, 2> tile = {x, y};
        tiles.push(tile);
    }
    void run() {
        while (running) {
            const std::array<int, 2>& tile = tiles.pop();
            io(tile[1], tile[2]);
        }
    }
    void wait() {
        for (const auto& t : tharr)
            t.wait();
    }
};

} // namespace atlaas

#endif // ATLAAS_TILES_WRITER_HPP
