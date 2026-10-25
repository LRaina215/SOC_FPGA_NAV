"""Review a full storage-only build with the existing strict RAM/clock engine."""
from pathlib import Path
import prepare_registered_action_storage as storage
import review_registered_action_csa_syn as registered

engine = registered.engine
preview = registered.preview
preview.CANDIDATE = storage.BASE/'action_registered_CSA_storage_candidate1'


def bind(source, prepared, proof):
    preview.bind(source, prepared, proof)
    assert proof['registered_product_storage_only_vs_SYN29']
    candidate = preview.CANDIDATE
    origin = engine.load(candidate/'prepared_review.json')
    assert origin['source_single_delta_vs_SYN29'] == storage.ACTION
    assert origin['constant_CSA_combinational_function_64bit_unchanged']
    assert origin['exact_sign_extension_before_original_rounding']
    assert origin['original_capture_and_publication_edges_unchanged']
    assert origin['product_storage_widths'] == storage.WIDTHS
    for row in origin['source_bindings']+origin['candidate_bindings']:
        assert engine.metadata(Path(row['path'])) == row
    exact = storage.transform((storage.SOURCE/storage.ACTION).read_text())
    assert (source/storage.ACTION).read_text() == exact
    for name in preview.CHANGED:
        if name != storage.ACTION:
            assert engine.same(source/name, storage.SOURCE/name)
    ranges = engine.load(candidate/'range_proof.json')
    assert ranges['passed'] and ranges['combinations'] == 32772
    assert ranges['ranges'] == storage.range_proof()['ranges']
    for row in (ranges['source_binding'], ranges['candidate_binding']):
        assert engine.metadata(Path(row['path'])) == row
    for root in (prepared, source):
        manifest = engine.load(root/'manifest.json')
        assert manifest['private_product_storage_exact']
        assert manifest['registered_product_storage_only_vs_SYN29']


def review(source, prepared, out):
    result = registered.review(source, prepared, out)
    actual = registered.actual_mapping
    old = engine.load(storage.BASE/'baseline_fusion_registered_CSA_one_shot/actual_SYN29_review_retry1.json')
    old_actual = old['registered_constant_CSA_actual_mapping']
    result['private_storage_contrast'] = dict(
        source_bits=118, previous_actual_product_FF=old_actual['actual_product_FF_count'],
        actual_product_FF=actual['actual_product_FF_count'],
        actual_product_FF_change=actual['actual_product_FF_count']-old_actual['actual_product_FF_count'],
        ranges_exact=True, arithmetic_and_output_edges_preserved=True,
        routing_and_50MHz_not_proven=True)
    engine.save(out, result)
    return result


engine.bind_action_contract = bind
engine.review = review
if __name__ == '__main__':
    raise SystemExit(engine.main())
