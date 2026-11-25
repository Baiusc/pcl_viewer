#!/bin/bash

sudo apt install autoconf libtool automake libc6-dbg splint curl git pkg-config gcc gdb cmake -y

ret_flag=0

HOME_PATH=$(pwd)
PKG_PATH="$(pwd)/pkg"
LIBUV_PATH="$PKG_PATH/libuv"
EXPAT_PATH="$PKG_PATH/expat-2.2.9"
EXPAT_PKG_PATH="$PKG_PATH/expat-2.2.9.tar.bz2"
DBUS_PATH="$PKG_PATH/dbus-1.12.0"
DBUS_PKG_PATH="$PKG_PATH/dbus-1.12.0.tar.gz"
GTEST_PATH="$PKG_PATH/googletest"
VALGRIND_PATH="$PKG_PATH/valgrind-3.23.0"
VALGRIND_PKG_PATH="$PKG_PATH/valgrind-3.23.0.tar.bz2"

install_libuv() {
    if [ ! -d "$LIBUV_PATH" ]; then
        git clone https://github.com/libuv/libuv.git "$LIBUV_PATH"
    else
        echo "libuv has been downloaded"
    fi

    if [ $? -eq 0 ]; then
        echo -e "\033[32m Downloaded libuv package successfully! \033[0m"
        cd "$LIBUV_PATH" || exit
        sh autogen.sh
        ./configure
        make
        make check
        sudo make install
        cd "$HOME_PATH" || exit
    else
        ret_flag=$((ret_flag + 1))
        echo -e "\033[31m Failed to download libuv package! \033[0m"
        exit
    fi
}

install_expat() {
    if [ ! -f "$EXPAT_PKG_PATH" ]; then
        curl --connect-timeout 30 -m 30 -fsSL https://distfiles.macports.org/expat/expat-2.2.9.tar.bz2 -o "$EXPAT_PKG_PATH"
    else
        echo "expat has been downloaded"
    fi

    if [ $? -eq 0 ]; then
        echo -e "\033[32m Downloaded expat package successfully! \033[0m"
        tar -xvf "$EXPAT_PKG_PATH" -C "$PKG_PATH"
        cd "$EXPAT_PATH" || exit
        ./configure
        make
        sudo make install
        cd "$HOME_PATH" || exit
    else
        ret_flag=$((ret_flag + 1))
        echo -e "\033[31m Failed to download expat package! \033[0m"
        exit
    fi
}

install_dbus() {
    if [ ! -f "$DBUS_PKG_PATH" ]; then
        curl --connect-timeout 30 -m 30 -fsSL https://dbus.freedesktop.org/releases/dbus/dbus-1.12.0.tar.gz -o "$DBUS_PKG_PATH"
    else
        echo "D-Bus has been downloaded"
    fi

    if [ $? -eq 0 ]; then
        echo -e "\033[32m Downloaded D-Bus package successfully! \033[0m"
        tar -xvf "$DBUS_PKG_PATH" -C "$PKG_PATH"
        cd "$DBUS_PATH" || exit
        ./configure
        make
        sudo make install
        sudo ln -s /usr/local/lib/libdbus-1.so /usr/local/lib/libdbus.so
        cd "$HOME_PATH" || exit
    else
        ret_flag=$((ret_flag + 1))
        echo -e "\033[31m Failed to download D-Bus package! \033[0m"
        exit
    fi
}

install_gtest() {
    if [ ! -d "$GTEST_PATH" ]; then
        git clone https://github.com/google/googletest.git "$GTEST_PATH"
    else
        echo "gtest has been downloaded"
    fi

    if [ $? -eq 0 ]; then
        echo -e "\033[32m Downloaded gtest package successfully! \033[0m"
        cd "$GTEST_PATH" || exit
        mkdir -p build && cd build/ || exit
        cmake ..
        make
        sudo make install
        cd "$HOME_PATH" || exit
    else
        ret_flag=$((ret_flag + 1))
        echo -e "\033[31m Failed to download gtest package! \033[0m"
        exit
    fi
}

install_valgrind() {
    if [ ! -f "$VALGRIND_PKG_PATH" ]; then
        curl --connect-timeout 30 -m 30 -fsSL https://sourceware.org/pub/valgrind/valgrind-3.23.0.tar.bz2 -o "$VALGRIND_PKG_PATH"
    else
        echo "Valgrind has been downloaded"
    fi

    if [ $? -eq 0 ]; then
        echo -e "\033[32m Downloaded Valgrind package successfully! \033[0m"
        tar -xvf "$VALGRIND_PKG_PATH" -C "$PKG_PATH"
        cd "$VALGRIND_PATH" || exit
        ./autogen.sh
        ./configure
        make -j8
        sudo make install
        cd "$HOME_PATH" || exit
    else
        ret_flag=$((ret_flag + 1))
        echo -e "\033[31m Failed to download Valgrind package! \033[0m"
        exit
    fi
}

install_pre_commit() {
    sudo apt install pre-commit clang-format clang-tidy cppcheck -y

    # clone pre-commit/pre-commit-hooks
    git clone https://github.com/pre-commit/pre-commit-hooks.git
    cd pre-commit-hooks || exit
    git checkout v5.0.0
    cd ..

    # clone pocc/pre-commit-hooks
    git clone https://github.com/pocc/pre-commit-hooks.git pocc-pre-commit-hooks
    cd pocc-pre-commit-hooks || exit
    git checkout v1.3.5
    cd ..

    pre-commit install
    mkdir -p ../.git/hooks

    mv pre-commit-hooks ../.git/hooks
    mv pocc-pre-commit-hooks ../.git/hooks
}

install_libuv
install_expat
install_dbus
install_gtest
install_valgrind
install_pre_commit

# end
if [ $ret_flag -eq 0 ]; then
    echo -e "\033[32m Environment setup successful! \033[0m"
else
    echo -e "\033[31m Environment setup failed! \033[0m"
fi
