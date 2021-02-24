light$ echo $DISPLAY
:0
light$ xauth list $DISPLAY
light/unix:0 MIT-MAGIC-COOKIE-1 076aaecfd370fd2af6bb9f5550b26926
light$ rlogin dark.matt.er
Password:
dark% setenv DISPLAY light.uni.verse:0
dark% xauth
Using authority file /home/zweije/.Xauthority
xauth> add light.uni.verse:0 . 076aaecfd370fd2af6bb9f5550b26926
xauth> exit
Writing authority file /home/zweije/.Xauthority
dark% xfig &
[15332]
dark% logout
light$