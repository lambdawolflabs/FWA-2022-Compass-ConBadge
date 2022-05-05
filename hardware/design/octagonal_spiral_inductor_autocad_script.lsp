;;; Octagonal Spiral Inductor Generator for AutoCAD
;;; 2019 Nick Winston
;;; Set your units beforehand! (typically in base millimeters)

(defun myerror (s) ; If an error (such as CTRL-C) occurs
; while this command is active...
(if (/= s "Function cancelled")
(princ (strcat "\nError: " s))
)
(setvar "cmdecho" ocmd) ; Restore saved modes
(setvar "blipmode" oblp)
(setq *error* olderr) ; Restore old *error* handler
(princ)
)

(defun octaspiral (bp turns cspacing cwidth innerdia / sp ep lone ltwo step ang temp denom top innerrad innerdiatwo spacestep)
  (setvar "blipmode" 0) ; turn off blipmode
  (setvar "cmdecho" 1) ; turn off cmdecho

  (setq sp 0)
  (setq ep 0)
  (setq lone 0)
  (setq ltwo 0)
  (setq step 0)
  (setq ang 0)
  (setq temp 0)
  (setq denom 0)
  (setq top 0)
  (setq innerrad 0)
  (setq innerdiatwo 0)
  (setq spacestep 0)
  
  (setq denom (sqrt (+ 4 (* 2 (sqrt 2)))))

  (setq denom (* denom 1.0))
  (setq spacestep (* 1.0823922 (+ cspacing cwidth)))
  

  
  (setq innerdia (* 1.0823922 innerdia))
  (setq innerdiatwo (+ innerdia spacestep))
  

  (setq lone (/ innerdia denom))

  (setq top (+ innerdiatwo))
  (setq ltwo (/ top denom))
  
  (setq step (- ltwo lone))
  (setq temp (/ lone (sqrt 2)))
  (setq innerrad (/ (+ lone temp temp) 2))

  (setq sp (list (+ innerrad (car bp)) (- (cadr bp) (/ lone 2)) (caddr bp)))
  (setq ang (/ 3.1415926535 2))


  (repeat turns
    (repeat 4
      (setq ep (polar sp ang lone))
      (command "._pline" sp ep "")
      (setq sp ep)


      (setq ang (+ ang (/ 3.1415926535 4)))
    )
    (repeat 4
      (setq ep (polar sp ang ltwo))
      (command "._pline" sp ep "")
      (setq sp ep)

      (setq ang (+ ang (/ 3.1415926535 4)))
    )

    (setq innerdia (+ innerdiatwo spacestep))
    (setq innerdiatwo (+ innerdia spacestep))
    (setq lone (/ innerdia denom))
    (setq ltwo (/ innerdiatwo denom))

    
  )


  (command "")
  
  (setq octaspiral nil)
  (setq c:octaspiral nil)
  (setq myerror nil)
)

(defun C:OCTASPIRAL (/ olderr ocmd oblp osnapang oortho bp turns cspacing cwidth innerdia)
  (setq olderr *error*
	*error* myerror)
  (setq ocmd (getvar "cmdecho"))
  (setq oblp (getvar "blipmode"))
  (setq osnapang (getvar 'snapang))
  (setq oortho (getvar 'orthomode))

  (setq bp 0)
  (setq turns 0)
  (setq cspacing 0)
  (setq cwidth 0)
  (setq innerdia 0)

  
  (setvar "cmdecho" 0)
  (initget 1) ;bp, basepoint must not be null
  (setq bp (getpoint "\nCenter Point: "))
  (initget 7) ;turns must not be zero, neg or null
  (setq turns (getint "\nTurns: "))
  ;(initget 7) ;cspacing, conductor spacing must not be zero, neg or null
  (setq cspacing (getreal "\nConductor Spacing: "))
  ;(initget 7) ;cwidth, conductor width must not be zero, neg or null
  (setq cwidth (getreal "\nConductor Width: "))
  ;(initget 7) ;innnerdia, inner diameter of spiral must not be zero, neg or null
  (setq innerdia (getreal "\nInner Diameter: "))
  (octaspiral bp turns cspacing cwidth innerdia)
  ;(octaspiral bp 15 0.18 0.18 11.06)
  (setvar "cmdecho" ocmd)
  (setvar "blipmode" oblp)
  (setvar 'snapang osnapang)
  (setvar 'orthomode oortho)
  (setq *error* olderr) ;Restore echo, blip and previous error
  (princ)
  (princ "\n\tC:OCTASPIRAL loaded. ")
  (princ)
)


