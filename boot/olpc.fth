\ File needs to start with "\ "

visible unfreeze

\ \ " /dcon-i2c@0/dcon@d" select-dev
\ \ backlight-off
\ \ unselect
\ 

" fload last:\boot\fixes.fth" evaluate
" fload last:\boot\dt.fth" evaluate
" fload last:\boot\menu.fth" evaluate
\ " fload http:\\v3.sk\~lkundrak\dt.fth" evaluate
