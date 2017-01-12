@echo off

REM change your filepath if you move these files
set filepath="C:\Users\David\Desktop\TennisBookerWinter"

cd %filepath%

REM arg 1 times [t1,t2,t3,...,tn] no spaces
REM	options are 10am, 10:30am,...., 4pm, 4:30pm, 5pm, 5:30pm, 6pm, 6:30pm, 7pm, 7:30pm,...,11pm 
REM arg 2 courts [c1,c2,c3, ...,cn] no spaces
REM 	options are 4, 5, 6, 7, 8, 9, 10, 11, 12
REM arg 3 is the name of the second player; lastname, firstname
REM arg 4 is the hour of the timer on a 24 hour clock
REM arg 5 is the minute of the timer

REM THESE ARE THE VARIABLES YOU CAN CHANGE
REM -------------------------------------------------------------
set times="[]"
set courts="[]"
set secondPlayer="O'Connor, David"
set hour=7
set minute=30
set second=0
set millisec=0
REM -------------------------------------------------------------

java -jar TennisBooker_winter.jar %times% %courts% %secondPlayer% %hour% %minute% %second% %millisec% > log.txt

pause