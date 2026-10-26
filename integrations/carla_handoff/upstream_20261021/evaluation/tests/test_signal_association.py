import unittest
from evaluation.signal_association import SignalAssociation


def row(inside=False,match="unique"):
    return dict(vehicle_id=1,current_junction_id=73 if inside else "",signal_match=match,signal_opendrive_id="A")


class SignalAssociationTest(unittest.TestCase):
    def test_retained_id_uses_live_color_and_clears_on_exit(self):
        matcher=SignalAssociation(73)
        matcher.update(1,[row()],{"A":"Red"})
        result=matcher.update(2,[row(True,"unmapped")],{"A":"Green"})["1"]
        self.assertEqual(result["approach_signal_id"],"A")
        self.assertEqual(result["signal_association_source"],"retained_entry_signal")
        self.assertEqual(result["approach_signal_state"],"Green")
        result=matcher.update(3,[row(False,"unmapped")],{"A":"Red"})["1"]
        self.assertIsNone(result["approach_signal_id"])

    def test_no_association_invented_across_gap_or_ambiguity(self):
        matcher=SignalAssociation(73)
        matcher.update(1,[row()],{"A":"Red"})
        self.assertIsNone(matcher.update(3,[row(True,"unmapped")],{})["1"]["approach_signal_id"])
        matcher.update(4,[row()],{})
        matcher.update(5,[row(False,"ambiguous")],{})
        self.assertIsNone(matcher.update(6,[row(True,"unmapped")],{})["1"]["approach_signal_id"])
