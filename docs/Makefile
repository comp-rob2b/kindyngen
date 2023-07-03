pandoc-pdf:
	@pandoc \
		--standalone \
		--filter pandoc-crossref \
		--citeproc \
		--top-level-division=section \
		--number-sections \
		-M reference-section-title=References \
		-V links-as-notes \
		-V linkcolor:blue \
		-f markdown+rebase_relative_paths \
		-t latex \
		-o ../gen/doc.pdf \
		introduction.md \
		installation.md \
		background.md \
		architecture.md \
		getting_started.md \
		tutorial_solver_configuration.md \
		tutorial_solver_robif.md \
		tutorial_controller.md

pandoc-merge:
	@pandoc \
		--standalone \
		--shift-heading-level-by=1 \
		-o ../gen/doc.md \
		introduction.md \
		installation.md \
		background.md \
		architecture.md \
		getting_started.md \
		tutorial_solver_configuration.md \
		tutorial_solver_robif.md \
		tutorial_controller.md