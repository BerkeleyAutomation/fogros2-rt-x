<script>
  export let active;
</script>

<div class={"modal" + (active ? " is-active" : "")}>
  <div
    class="modal-background"
    style="opacity:0.7"
  ></div>
    <!-- on:click={() => (active = false)} -->
  <div class="modal-content">
    <div class="card p-3">
      <card class="header">
        <slot />
      </card>
    </div>
  </div>
</div>
