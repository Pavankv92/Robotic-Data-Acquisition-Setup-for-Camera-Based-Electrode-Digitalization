\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{mtecreprt}
\RequirePackage{kvoptions}
\DeclareBoolOption[false]{nobcorignoretitle}
\DeclareBoolOption[false]{grayscalebody}
\DeclareBoolOption[false]{grayscaletitle}
\DeclareBoolOption[false]{fancyheadings}
\DeclareStringOption[scrbook]{baseclass}
\ProcessKeyvalOptions*
\LoadClassWithOptions{\mtecreprt@baseclass}


\ifmtecreprt@fancyheadings
\RequirePackage[automark,headsepline,footsepline]{scrlayer-scrpage}
\else
\RequirePackage[automark]{scrlayer-scrpage}
\fi
\RequirePackage{amsmath,amsfonts,amssymb}
\RequirePackage{xstring}
\RequirePackage{graphicx}
\RequirePackageWithOptions{xcolor}
\RequirePackage{titlepage}
\TitlePageStyle{TUHH-MTEC}

\RequirePackage[pdfpagelabels,plainpages=false]{hyperref}

\RequirePackage{float}
\floatstyle{plaintop}
\restylefloat{table}

\pagestyle{scrheadings}
\ifmtecreprt@fancyheadings
	\newcommand{\twosideheadings}{\rehead{\leftmark}\lohead{\rightmark}}
	\newcommand{\onesideheadings}{\ihead{\leftmark}\chead{}}

	\ifmtecreprt@grayscalebody
		\ohead{\setlength{\unitlength}{0.75cm}\begin{picture}(1,0)\put(0,0){\includegraphics[width=0.75cm]{logo/logo_mtec_letters_gray}}\end{picture}}
	\else
		\ohead{\setlength{\unitlength}{0.75cm}\begin{picture}(1,0)\put(0,0){\includegraphics[width=0.75cm]{logo/logo_mtec_letters}}\end{picture}}
	\fi
	
	\makeatletter
	\if@twoside\twosideheadings \else \onesideheadings \fi
	\makeatother
	
	\ifx\chapterpagestyle\undefined\else\renewcommand*{\chapterpagestyle}{scrheadings}\fi
	\ifx\partpagestyle\undefined\else\renewcommand*{\partpagestyle}{scrheadings}\fi
	\ifx\indexpagestyle\undefined\else\renewcommand*{\indexpagestyle}{scrheadings}\fi
	\ifx\chapterheadstartvskip\undefined\else\renewcommand{\chapterheadstartvskip}{\vspace *{2.3\baselineskip }}\fi
	\ifx\chapterheadmidvskip\undefined\else\renewcommand{\chapterheadmidvskip}{\par\nobreak\vskip.5\baselineskip}\fi
	\ifx\chapterheadendvskip\undefined\else\renewcommand{\chapterheadendvskip}{\vspace*{1.725\baselineskip plus .115\baselineskip minus .192\baselineskip}}\fi

\else
\fi
\addtokomafont{disposition}{\rmfamily}

\AtBeginDocument{
	\@ifpackagewith{babel}{english}{%
		\addto\captionsenglish{%
			\renewcommand{\figurename}{Fig.}%
			\renewcommand{\tablename}{Tab.}%
		}%
		\Crefname{equation}{Eq.}{Eqs.}%
		\Crefname{figure}{Fig.}{Figs.}%
		\Crefname{tabular}{Tab.}{Tabs.}%
		\Crefname{table}{Tab.}{Tabs.}%
		\Crefname{section}{Sec.}{Secs.}%
	}{}
	\@ifpackagewith{babel}{german}{%
		\addto\captionsgerman{%
			\renewcommand{\figurename}{Abb.}%
			\renewcommand{\tablename}{Tab.}%
		}%
		\Crefname{equation}{Gl.}{Gln.}%
		\Crefname{figure}{Abb.}{Abb.}%
		\Crefname{tabular}{Tab.}{Tab.}%
		\Crefname{table}{Tab.}{Tab.}%
		\Crefname{section}{Abs.}{Abs.}%
	}{}
	\@ifpackagewith{babel}{ngerman}{%
		\addto\captionsngerman{%
			\renewcommand{\figurename}{Abb.}%
			\renewcommand{\tablename}{Tab.}%
		}%
		\Crefname{equation}{Gl.}{Gln.}%
		\Crefname{figure}{Abb.}{Abb.}%
		\Crefname{tabular}{Tab.}{Tab.}%
		\Crefname{table}{Tab.}{Tab.}%
		\Crefname{section}{Abs.}{Abs.}%
	}{}
}

\RequirePackage{cleveref}


\endinput
