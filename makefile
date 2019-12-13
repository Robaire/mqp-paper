
FILES = format/meta.yaml sections/introduction.md sections/background.md sections/analysis.md sections/results.md sections/references.md sections/appendix.md
FLAGS = --filter=pandoc-xnos --filter=pandoc-citeproc --bibliography=sections/bibliography.bib --csl=format/ieee.csl --template=format/template.tex

all: report

report:
	pandoc $(FLAGS) $(FILES) -o report.pdf

clean:
	rm report.pdf


