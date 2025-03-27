$(document).ready(function () {
  let images = [];
  let currentIndex = 0;

  $(".drop-area")
    .on("dragover", function (e) {
      e.preventDefault();
      $(this).addClass("dragging");
    })
    .on("dragleave", function () {
      $(this).removeClass("dragging");
    })
    .on("drop", function (e) {
      e.preventDefault();
      $(this).removeClass("dragging");
      handleFiles(e.originalEvent.dataTransfer.files);
    });

  $("#file-input").on("change", function (e) {
    handleFiles(e.target.files);
  });

  function handleFiles(files) {
    Array.from(files)
      .filter((file) => file.type.startsWith("image/"))
      .forEach((file) => {
        const reader = new FileReader();
        reader.onload = (e) => {
          images.push(e.target.result);
          updateCarousel();
        };
        reader.readAsDataURL(file);
      });
  }

  function updateCarousel() {
    const $carouselInner = $(".carousel-inner");
    $carouselInner.empty();

    images.forEach((src) => {
      $carouselInner.append(`<img src="${src}" class="carousel-image" />`);
    });

    $carouselInner.css("transform", `translateX(-${currentIndex * 100}%)`);
  }

  $(".carousel-control.prev").on("click", function () {
    if (images.length > 0) {
      currentIndex = (currentIndex - 1 + images.length) % images.length;
      updateCarouselPosition();
    }
  });

  $(".carousel-control.next").on("click", function () {
    if (images.length > 0) {
      currentIndex = (currentIndex + 1) % images.length;
      updateCarouselPosition();
    }
  });

  function updateCarouselPosition() {
    console.log(currentIndex);
    $(".carousel-inner").css(
      "transform",
      `translateX(-${currentIndex * 100}%)`
    );
  }
});
