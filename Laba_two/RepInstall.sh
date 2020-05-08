#!/bin/bash

# Лабораторная работа №2

check_dir() {
	
	# Проверяет введенный пользователем путь до целевого каталога
	
	if [ -z "$1" ] ; then
		echo "Folder will be created in current working directory"
		rep_dir="$(pwd)/cjdns"
	else
		if [ -d "$1" ] ; then
			rep_dir="$1/cjdns"
		elif [ -f "$1" ] ; then
			echo "Error: Expected directory, not file"
			exit 1
		else
			echo "Error: No such directory"
			exit 1
		fi
	fi		 
}

check_pack() {
	# Проверяет, установлены ли пакеты в систему

	all_pack=1
	for pack in ${packages[@]}; do
		if [ -z "$(dpkg --get-selections | grep "\b$pack")" ] ; then
			echo "Error: Package $pack not found"
			all_pack=0
		fi
	done
	
	# Установка необходимых пакетов
	if (( $all_pack==0 )) ; then
		echo "Enter your password to install needed packages"
		sudo apt-get install -y "${packages[@]}"
	fi
}


######################
# ОСНОВНАЯ ПРОГРАММА #
######################

cur_dir="$(pwd)"
# Пакеты, необходимые для установки
packages=("git" "python2.7" "build-essential")

echo "Hello, it's cjdns project installer."
check_dir "$1"
check_pack $packages

git clone https://github.com/cjdelisle/cjdns.git "$rep_dir"
# Запуск компиляции
echo "Start compiling the program"
cd $rep_dir && ./do > /dev/null
echo

# Проверка корректности установки
echo "Test the program"
LANG=C cat /dev/net/tun
echo "If there is \"No such file or directory\" message, enter 1"
echo "Else if there is \"Permission denied\" message, enter 2"
echo "Else enter 3"
read mes

if [[ "$mes" =~ ^1.*$ ]] ; then
	echo "There is no neccessary file or directory. They will be created automatically"
	sudo mkdir -p /dev/net && sudo mknod /dev/net/tun c 10 200 && sudo chmod 0666 /dev/net/tun
elif [[ "$mes" =~ ^2.*$ ]] ; then
	echo "Error: Permission denied"
	exit 1
else
	echo "Installing is completed"
fi

# Вывод инструкций по дальнейшим действиям
echo "There are some instructions about next steps"
./cjdroute

# Возвращение в рабочую директорию
cd "$cur_dir" > /dev/null
exit 0
