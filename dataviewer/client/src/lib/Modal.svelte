<script>
    // export let onConfirm;
    // export let onCancel;
    export let active;
    // export let text;
</script>

<div class={"modal" + (active ? " is-active" : "")}>
    <div class="modal-background" style="opacity:0.7"></div>
    <div class="modal-content">
        <div class="card p-3">
            <card class="header">
                <slot>

                </slot>
            </card>
        </div>
    </div>
    <button class="modal-close is-large" aria-label="close"></button>
  </div>