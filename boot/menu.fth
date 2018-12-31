\ Boot menu

variable entries

: heapstr
   dup alloc-mem swap 3dup move rot drop
;

: bootentry
   entries @ .
   type cr
   entries dup @ 1 + swap !
;

: bootselect
   \ Display the counter
   0 swap \ Default to zero
   1 swap
   do
      i .d " seconds left to make a choice... " type
      d# 10 0 do
         key? if
            drop
            key 30 - \ ord(30) = '0'
            dup entries @ u>= if drop 0 then \ Turn to zero if out of range
            unloop leave
         then
         d# 100 ms
      loop
      (cr
   -1 +loop

   cr " Booting entry: " type dup . cr

   \ Set the boot entry
   entries @ swap do
      2dup to ramdisk
      free-mem
      2dup to boot-file
      free-mem
      2dup to boot-device
      free-mem
      entries dup @ 1 - swap !
   loop

   \ Discard the rest
   entries @ 0 > if
      entries @ 0 do
         free-mem
         free-mem
         free-mem
      loop
   then

   boot
;

cr

" External boot" bootentry
" ext:\boot\zImage" heapstr
" " heapstr
" " heapstr


" Fedora" bootentry
" ext:\boot\vmlinuz-4.20.0-0.rc7.git1.1.lr1.fc29.armv7hl" heapstr
" console=tty0 console=ttyS2,115200 earlyprintk root=/dev/mmcblk1p1 rootwait" heapstr
" ext:\boot\initramfs-4.20.0-0.rc7.git1.1.lr1.fc29.armv7hl.img" heapstr


" External boot (good)" bootentry
" ext:\boot\zImage-works" heapstr
" " heapstr
" " heapstr

" No appended DTB (good)" bootentry
" ext:\boot\zImage-works-bare" heapstr
" console=tty0 console=ttyS2,115200 earlyprintk root=/dev/mmcblk1p1 rootwait" heapstr
" " heapstr

" Local boot" bootentry
" int:\boot\olpc.fth" heapstr
" " heapstr
" " heapstr

\ " Single user mode" bootentry
\ " int:\boot\olpc.fth" heapstr
\ " init=/bin/bash console=ttyS2,115200" heapstr
\ " " heapstr
\ 
\ " Experimental" bootentry
\ " int:\boot\zImage2" heapstr
\ " console=ttyS2,115200 rd.shell" heapstr
\ " int:\boot\initrd2.img" heapstr
\ 
\ " Old kernel" bootentry
\ " ext:\boot\zImage-works" heapstr
\ " " heapstr
\ " " heapstr

" Network boot" bootentry
" http:\\v3.sk\~lkundrak\xo175\vmlinuz" heapstr
" console=tty0 console=ttyS2,115200 earlyprintk root=/dev/mmcblk0p1 rootwait" heapstr
" " heapstr

" Dump FDT" bootentry
" ext:\boot\fdt.fth" heapstr
" " heapstr
" " heapstr


\ " " heapstr
\ " console=ttyS2,115200 root=LABEL=OLPCRoot rd.shell" heapstr
\ " int:\boot\initrd.img" heapstr

cr

d# 5 bootselect
