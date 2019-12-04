# Introduction
This is all written in what is essentially Markdown, with some minor additional features. I recommend reading up on Markdown to understand what is going on. The additional features are explained in the sections below. Additionally, a tool called Pandoc is used to convert this text into a pdf. Pandoc can be very complicated and at times it may be difficult to understand what it is doing, but I recommend you skim its documentation regardless.

# Formatting Citations
Sources are added to the `bibliography.bib` file in the sections folder. Online tools like easybib can export citations in the bibtex format so that they may be copied directly in. When citing a source in text use `[@citation-name]` where the citation should be inserted. When the document is processed into a pdf this citation will be automatically filled, like this [@sec].

# Formatting Equations
Number and reference equations works in much the same way. In line equations are surrounded by `$`. Block equations are denoted with `$$`. For example, `$$ y = mx + b $${#eq:slope-intercept}`. The text in the curly braces allows for the equation to be easily referenced later in the text.

$$ y = mx + b $${#eq:slope-intercept}

Now that the equation has been defined in the paper I can make a reference to it much the same way citations are created. By writing `[@eq:slope-intercept]` a reference number is created automatically, like this [@eq:slope-intercept].

# Formatting Images
Images are denoted by `![Image Title](./images/image.png){#fig:logo width=50%}`. You will notice that much like the tag used to denote equations images have a similar tag, but replaces `eq` with `fig`. 

![Image Title](./images/logo.png){#fig:logo width=50%}

The image can now be referenced in the text using `[@fig:logo]`, like it is at the end of this line [@fig:logo].

# Closing
These are just the basics of formatting things, but it should be enough to get started for the time being.  Look at the pdf and raw text versions side by side to get a better understanding of what is happening.

# References
