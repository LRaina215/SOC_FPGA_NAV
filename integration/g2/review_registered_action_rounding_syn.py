"""Strict full-build audit for the same-edge private rounded value candidate."""
from pathlib import Path
import prepare_registered_action_rounding as rounding
import review_registered_action_csa_syn as registered

engine = registered.engine
preview = registered.preview
preview.CANDIDATE = rounding.BASE/'action_registered_rounding_candidate1'


def bind(source, prepared, proof):
    preview.bind(source, prepared, proof)
    assert proof['private_rounding_relocation_only_vs_SYN30']
    candidate = preview.CANDIDATE
    origin = engine.load(candidate/'prepared_review.json')
    assert origin['source_single_delta_vs_SYN30'] == rounding.ACTION
    assert origin['original64bit_CSA_and_rounding_functions_unchanged']
    assert origin['private_capture_edge_and_external_output_edges_unchanged']
    assert origin['private_rounded_product_widths'] == rounding.WIDTHS
    for row in origin['source_bindings']+origin['candidate_bindings']:
        assert engine.metadata(Path(row['path'])) == row
    assert (source/rounding.ACTION).read_text() == rounding.transform((rounding.SOURCE/rounding.ACTION).read_text())
    for name in preview.CHANGED:
        if name != rounding.ACTION:
            assert engine.same(source/name, rounding.SOURCE/name)
    ranges = engine.load(candidate/'range_proof.json')
    assert ranges['passed'] and ranges['ranges'] == rounding.range_proof()['ranges']
    for row in (ranges['source_binding'], ranges['candidate_binding']):
        assert engine.metadata(Path(row['path'])) == row
    for root in (source, prepared):
        manifest = engine.load(root/'manifest.json')
        assert manifest['private_rounding_relocation_only_vs_SYN30']
        assert manifest['private_rounded_product_capture']


def review(source, prepared, out):
    result = registered.review(source, prepared, out)
    mapping = registered.actual_mapping
    result['private_rounded_capture_actual_mapping'] = dict(
        actual_FFs=mapping['actual_product_FF_count'],
        groups={g:len(v['registers']) for g,v in mapping['actual_product_groups'].items()},
        source_bits=46, previous_unrounded_actual_FFs=112,
        captured_quantity='exact original rounded product on original E2 edge',
        data_clock_and_async_controls_audited=True,
        original_external_cycle_and_latest_feedforward_preserved=True,
        full_physical_timing_not_proven=True)
    engine.save(out, result)
    return result


engine.bind_action_contract = bind
engine.review = review
if __name__ == '__main__':
    raise SystemExit(engine.main())
