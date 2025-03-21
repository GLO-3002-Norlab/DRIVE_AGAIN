$(document).ready(() => {
  $("#select-option").change((e) => {
    $("#section-1").hide();
    $("#section-2").hide();
    $("#section-3").hide();

    if (e.target.value === "1") {
      $("#section-1").show();
    }
    if (e.target.value === "2") {
      $("#section-2").show();
    }
    if (e.target.value === "3") {
      $("#section-3").show();
    }
  });

  let options = [
    { value: "1", text: "Ooga" },
    { value: "2", text: "Booga" },
  ];

  $.each(options, (index, option) => {
    $("#dynamic-select").append(
      $("<option>", {
        value: option.value,
        text: option.text,
      })
    );
  });
});
