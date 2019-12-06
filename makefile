
FILES = format/meta.yaml sections/introduction.md sections/background.md sections/analysis.md sections/results.md sections/references.md
FLAGS = --filter=pandoc-xnos --filter=pandoc-citeproc --bibliography=sections/bibliography.bib --csl=format/apa.csl --template=format/template.tex

all: report

help:
	pandoc $(FLAGS) format/meta.yaml help.md -o help.pdf

report:
	pandoc $(FLAGS) $(FILES) -o report.pdf

clean:
	rm report.pdf


