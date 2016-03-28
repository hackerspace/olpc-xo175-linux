\ OLPC boot script

[ifdef] require-signatures?
   : no-signatures  ( -- )  false to require-signatures?  ;
[else]
   : is-valid  ( $ $ -- true )  4drop true  r> drop  ;
   : is-leased  ( -- )  " run" cn-buf place  ;
   : no-signatures  ( -- )
      ['] is-valid ['] fw-valid?  >body token!
      ['] is-valid ['] sha-valid? >body token!
      ['] false      ['] has-developer-key?  ['] load-from-list    (patch

      ['] is-leased  ['] ?leased             ['] load-from-device  (patch
   ;
[then]

: set-path-macros  ( -- )
   button-o game-key?  if  " \boot-alt"  else  " \boot"  then  pn-buf place

   " /chosen" find-package  if                       ( phandle )
      " bootpath" rot  get-package-property  0=  if  ( propval$ )
         get-encoded-string                          ( bootpath$ )
         [char] \ left-parse-string  2nip            ( dn$ )
         dn-buf place                                ( )
      then
   then
;

: unsigned-boot  ( -- )
   no-signatures
   alternate?  if  " \boot-alt"  else  " \boot"  then  pn-buf place
   " last:" load-from-list drop
;

: olpc-fth-boot-me
   set-path-macros
   " ${DN}${PN}\vmlinuz" expand$ 2dup $file-exists? if
      to boot-device
      " ${DN}${PN}\initrd.img" expand$ to ramdisk
      \ " extra kernel parameters here" to boot-file
      boot
   else
      2drop
      unsigned-boot
   then
;

olpc-fth-boot-me
