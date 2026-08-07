# Host-side tests for the pure decision core (src/ac_decision.*).
# No framework, no hardware: plain g++.
CXX ?= g++
CXXFLAGS = -std=c++17 -Wall -Wextra -Werror -g -fsanitize=address,undefined -I../src

test_decide: test_decide.cpp ../src/ac_decision.cpp ../src/ac_decision.h
	$(CXX) $(CXXFLAGS) test_decide.cpp ../src/ac_decision.cpp -o $@ -lm

.PHONY: run clean check-ref
run: check-ref test_decide
	./test_decide

# The oracle's step-down machinery (only — see the decide_reference note in
# test_decide.cpp) descends from ref_extract_a805f40.txt. Re-extract from git
# and diff, so silent drift from the pinned commit fails the run. Skipped
# gracefully outside a git checkout.
check-ref:
	@if git -C .. rev-parse a805f40 >/dev/null 2>&1; then \
	  { git -C .. show a805f40:src/ac_controller.cpp | sed -n '375,410p'; \
	    echo '/* ---- fan block ---- */'; \
	    git -C .. show a805f40:src/ac_controller.cpp | sed -n '437,464p'; } \
	    | diff -u ref_extract_a805f40.txt - \
	    && echo "check-ref: extraction matches a805f40"; \
	else echo "check-ref: skipped (no git)"; fi

clean:
	rm -f test_decide
