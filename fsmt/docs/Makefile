# Minimal makefile for Sphinx documentation
#

# You can set these variables from the command line, and also
# from the environment for the first two.
SPHINXOPTS    ?=
SPHINXBUILD   ?= sphinx-build
SOURCEDIR     = .
BUILDDIR      = _build

# Put it first so that "make" without argument is like "make help".
help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile

# Catch-all target: route all unknown targets to Sphinx using the new
# "make mode" option.  $(O) is meant as a shortcut for $(SPHINXOPTS).
%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)


# DOXYGEN = doxygen
# SPHINX_BUILD = sphinx-build
# DOXYFILE = Doxyfile
# BUILDDIR = _build

# .PHONY: all doxygen sphinx clean

# # Run both Doxygen and Sphinx
# all: doxygen sphinx

# # Generate Doxygen documentation
# doxygen:
# 	$(DOXYGEN) $(DOXYFILE)

# # Build Sphinx documentation
# sphinx:
# 	$(SPHINX_BUILD) -b html . $(BUILDDIR)/html

# # Clean old docs
# clean:
# 	rm -rf $(BUILDDIR) doxygen_output