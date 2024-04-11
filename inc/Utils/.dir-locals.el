;;; Directory Local Variables
;;; For more information see (info "(emacs) Directory Variables")

((c++-mode . (
              (c-basic-offset . 4)
  (flycheck-checker . c/c++-gcc)
  (flycheck-clang-standard-library . "libc++")
  (flycheck-clang-include-path . ("../"))
  (flycheck-gcc-include-path . ("/opt/homebrew/include" "../"))
  (flycheck-clang-language-standard . "c++14")
  (flycheck-gcc-language-standard . "c++14")
  )

           )

 (nil . (
         (eval . (add-to-list 'auto-mode-alist '("\\.h\\'" . c++-mode)))
         ))

 )




