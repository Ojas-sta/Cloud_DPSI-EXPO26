"""
Project Cloud: Grounded Exhibition RAG & Fact Retrieval Engine
Indexes curated technical curricula for all 8 DPSI EXPO student exhibits.
Enforces strict hallucination guardrails by answering exclusively from verified facts.
Reference: PROJECT.md Section 7.4 & Appendix E
"""

import os
import yaml
import re
from typing import Dict, List, Optional, Tuple, Any

class GroundedRAGEngine:
    """
    Offline local retrieval-augmented generation engine for exhibition booths.
    """
    def __init__(self, curricula_path: Optional[str] = None):
        if curricula_path is None:
            base_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
            curricula_path = os.path.join(base_dir, "config", "exhibition_curricula.yaml")

        self.curricula_path = curricula_path
        self.curricula: Dict[str, Any] = {}
        self.marker_to_key: Dict[int, str] = {}
        self._load_yaml()

    def _load_yaml(self):
        if not os.path.exists(self.curricula_path):
            raise FileNotFoundError(f"Exhibition curricula file missing: {self.curricula_path}")

        with open(self.curricula_path, "r", encoding="utf-8") as f:
            self.curricula = yaml.safe_load(f)

        for key, data in self.curricula.items():
            marker_id = data.get("marker_id")
            if marker_id is not None:
                self.marker_to_key[marker_id] = key

    def get_curriculum(self, key_or_marker: Any) -> Optional[Dict[str, Any]]:
        if isinstance(key_or_marker, int):
            key = self.marker_to_key.get(key_or_marker)
            return self.curricula.get(key) if key else None
        return self.curricula.get(key_or_marker)

    def get_narration(self, key_or_marker: Any) -> str:
        """Retrieves the official spoken presentation script for the landmark."""
        curr = self.get_curriculum(key_or_marker)
        if not curr:
            return "Welcome to DPSI EXPO 2026. Please observe the student demonstration."
        return curr.get("narration_script") or curr.get("greeting_script") or curr.get("conclusion_script") or ""

    def answer_question(self, key_or_marker: Any, question: str) -> Tuple[str, float]:
        """
        Retrieves grounded answer for a visitor's question about the active booth.
        Returns (answer_text, confidence).
        """
        curr = self.get_curriculum(key_or_marker)
        if not curr:
            return (
                "I do not have technical records for this location in my active database.",
                0.0
            )

        q_clean = question.lower()
        q_tokens = set(re.findall(r"\w+", q_clean))

        best_answer = None
        best_score = 0.0

        # Check curated Q&A pairs
        qa_pairs = curr.get("qa_pairs", [])
        for pair in qa_pairs:
            candidate_q = pair.get("q", "").lower()
            candidate_tokens = set(re.findall(r"\w+", candidate_q))

            if candidate_tokens and q_tokens:
                overlap = len(candidate_tokens.intersection(q_tokens))
                union = len(candidate_tokens.union(q_tokens))
                jaccard = overlap / union
                bonus = 0.3 if any(w in candidate_q for w in q_tokens if len(w) > 3) else 0.0
                score = jaccard + bonus

                if score > best_score:
                    best_score = score
                    best_answer = pair.get("a", "")

        # Check key specifications dictionary
        key_specs = curr.get("key_specs", {})
        for spec_key, spec_val in key_specs.items():
            spec_tokens = set(re.findall(r"\w+", spec_key.lower()))
            overlap = len(spec_tokens.intersection(q_tokens))
            if overlap > 0:
                score = 0.6 + 0.2 * (overlap / len(spec_tokens))
                if score > best_score:
                    best_score = score
                    best_answer = f"The {spec_key.replace('_', ' ')} is {spec_val}."

        # Threshold check: requires at least 0.25 confidence for grounded answer
        if best_answer and best_score >= 0.25:
            return (best_answer, min(1.0, best_score))

        # Anti-hallucination fallback
        booth_title = curr.get("title", "this project")
        fallback = (
            f"I do not have verified technical specifications for that question regarding {booth_title}. "
            "I invite you to speak directly with the student engineering team present at this booth."
        )
        return (fallback, 0.0)
